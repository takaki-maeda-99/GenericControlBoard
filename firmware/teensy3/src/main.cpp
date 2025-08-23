#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "DjiMotor.hpp"
#include "PID.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan motors(can2, 129.5f);

volatile int state = 0;
volatile int buttons = 0;
volatile int lx = 0, ly = 0;
volatile int rx = 0, ry = 0;
volatile int l2 = 0, r2 = 0;

struct buttons_t {
    bool square;
    bool cross;
    bool circle;
    bool triangle;
    bool l1;
    bool r1;
    bool options;
    bool share;
} button;

uint8_t myBoardId = 3; // teensy3 Board ID = 3

const float homeAngles[4] = {-PI+0.05,-PI+0.05,-PI+0.05,-PI};

// Homing state management
enum HomingState {
    HOMING_IDLE,
    HOMING_IN_PROGRESS,
    HOMING_COMPLETED
};

struct HomingData {
    HomingState state;
    uint32_t startTime;
} homingData[4];

// Homing parameters
const int16_t HOMING_CURRENT = -1500;        // mA (negative direction)
const uint32_t HOMING_TIME = 2000;          // ms (time to apply current)

// Frame 1: state + mode + sticks (6 bytes)
struct ControllerFrame1 {
  uint8_t state;           // 1 byte
  uint8_t mode;            // 1 byte
  int8_t lx, ly;          // 2 bytes
  int8_t rx, ry;          // 2 bytes
} __attribute__((packed));

// Frame 2: buttons + triggers (4 bytes)
struct ControllerFrame2 {
  uint16_t buttons;        // 2 bytes
  uint8_t l2, r2;         // 2 bytes (full precision)
} __attribute__((packed));

// teensy3 processes controller data


// teensy3 CAN command reception

// CAN controller data reception
ControllerFrame1 controllerFrame1 = {0};
ControllerFrame2 controllerFrame2 = {0};
bool controllerDataReceived = false;

// teensy3 processes controller data


struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};

// Motor PID parameters
constexpr PidParam MotorAnglePidParam{20, 0, 0, -30, 30, 10};     // Angle->Speed cascade
constexpr PidParam MotorSpeedPidParam{500, 0, 0, -8000, 8000, 1};  // Speed->Current

// Utility function for PID creation
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID object arrays (4 motors each)
Pid MotorAnglePid[4] = { makePID(MotorAnglePidParam), makePID(MotorAnglePidParam), makePID(MotorAnglePidParam), makePID(MotorAnglePidParam)};
Pid MotorSpeedPid[4] = { makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam)};

#define PI 3.14159265358979323846

inline void motorSpeedControl(size_t idx, float targetSpeed) {
    const auto &fb = motors.feedback(idx + 1);
    
    int16_t speedCmd = MotorSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}

inline void motorAngleControl(size_t idx, float targetAngle, bool bounded) {
    const auto &fb = motors.feedback(idx + 1);
    const float fbAngle = fb.getAngleRadiansWrapped();
    const float fbSpeed = fb.getSpeedRadiansPerSec();
    
    if (bounded) {
        // Bounded angle: clamp target to -π ~ +π
        targetAngle = constrain(targetAngle, -PI, PI);
    } else {
        // Unbounded angle: adjust target for shortest path
        if (targetAngle - fbAngle > PI) {
            targetAngle -= 2*PI;
        } else if (targetAngle - fbAngle < -PI) {
            targetAngle += 2*PI;
        }
    }
    
    // Cascade control: Angle->Speed->Current
    float targetSpeed = MotorAnglePid[idx].compute(fbAngle, targetAngle);
    int16_t speedCmd = MotorSpeedPid[idx].compute(fbSpeed, targetSpeed);
    
    motors.sendCurrent(idx + 1, speedCmd);
}

void initializeHoming() {
    for (int i = 0; i < 4; i++) {
        homingData[i].state = HOMING_IDLE;
        homingData[i].startTime = 0;
    }
}

void startHoming(uint8_t motorIdx) {
    if (motorIdx >= 4) return;
    
    // Check if motor feedback is available before starting
    const auto &fb = motors.feedback(motorIdx + 1);
    if (fb.isTimeout) {
        Serial.printf("Motor %d: No feedback - cannot start homing\n", motorIdx + 1);
        return;
    }
    
    homingData[motorIdx].state = HOMING_IN_PROGRESS;
    homingData[motorIdx].startTime = millis();
    
    Serial.printf("Starting homing for motor %d (feedback confirmed)\n", motorIdx + 1);
}

bool updateHoming(uint8_t motorIdx) {
    if (motorIdx >= 4 || homingData[motorIdx].state != HOMING_IN_PROGRESS) {
        return false;
    }
    
    uint32_t currentTime = millis();
    
    // Check if homing time has elapsed
    if (currentTime - homingData[motorIdx].startTime >= HOMING_TIME) {
        // Homing complete - stop motor and reset angle to home position
        Serial.printf("Homing completed for motor %d (time elapsed)\n", motorIdx + 1);
        
        motors.sendCurrent(motorIdx + 1, 0);
        motors.resetAngle(motorIdx + 1, homeAngles[motorIdx]);
        
        homingData[motorIdx].state = HOMING_COMPLETED;
        return true;
    }
    
    // Continue applying homing current
    motors.sendCurrent(motorIdx + 1, HOMING_CURRENT);
    
    return false; // Homing still in progress
}

bool isHomingComplete(uint8_t motorIdx) {
    if (motorIdx >= 4) return false;
    return homingData[motorIdx].state == HOMING_COMPLETED;
}

struct MotorCommand {
    uint8_t motorId;     // Motor ID (1-8)
    uint8_t mode;        // 0x00=Disable, 0x01=Speed, 0x02=BoundedAngle, 0x03=UnboundedAngle
    uint8_t opt;         // Future expansion/flags (currently 0)
    int32_t value;       // Command value (rad/s or rad)
    uint8_t seq;         // Sequence number (0-255 cyclic)
    uint8_t timeout;     // Timeout [10ms units] (0=board default)
    uint32_t lastUpdate; // Last update time [ms]
    
    MotorCommand() : motorId(0), mode(0x00), opt(0), value(0), seq(0), timeout(0), lastUpdate(0) {}
} __attribute__((packed));

// Latest command for each motor
MotorCommand motorCommands[8];

bool parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd) {
    // Extract BoardId and MotorId from CAN message ID
    uint8_t boardId = (msg.id >> 4) & 0x0F;  // Upper 4 bits
    uint8_t motorId = msg.id & 0x0F;         // Lower 4 bits
    
    // Not for this board, or motor ID out of range
    if (boardId != myBoardId || motorId < 1 || motorId > 8) {
        return false;
    }
    
    // CAN data length check
    if (msg.len < 8) {
        return false;
    }
    
    // Convert CAN message to struct
    cmd.motorId = motorId;
    cmd.mode = msg.buf[0];
    cmd.opt = msg.buf[1];
    
    // Restore int32 value in little endian
    cmd.value = (int32_t)msg.buf[2] | 
                ((int32_t)msg.buf[3] << 8) | 
                ((int32_t)msg.buf[4] << 16) | 
                ((int32_t)msg.buf[5] << 24);
    
    cmd.seq = msg.buf[6];
    cmd.timeout = msg.buf[7];
    cmd.lastUpdate = millis();
    
    return true;
}

float convertToFloat(int32_t value) {
    // int32 to float conversion (fixed point assumption: 1000x scale)
    return (float)value / 1000.0f;
}

static void handleCANMessage(const CAN_message_t &msg) {
    // Motor command processing only (0x03X)
    MotorCommand cmd;
    if (parseMotorCommand(msg, cmd)) {
        uint8_t idx = cmd.motorId - 1;  // 0-7 index
        
        // Allow homing command (0xFF) even if not homed
        if (cmd.mode != 0xFF && !isHomingComplete(idx)) {
            Serial.printf("Motor%d: Command blocked - not homed\n", cmd.motorId);
            return; // Block commands until homing complete
        }
        
        // Sequence number check (reject duplicate commands)
        if (cmd.seq == motorCommands[idx].seq && motorCommands[idx].lastUpdate > 0) {
            return; // Ignore same sequence number
        }
        
        // Update command
        motorCommands[idx] = cmd;
        
        Serial.printf("Motor%d: mode=%02X, val=%.3f, seq=%d\n", 
                     cmd.motorId, cmd.mode, convertToFloat(cmd.value), cmd.seq);
    }
}

IntervalTimer motorControlTimer;

bool isCommandValid(const MotorCommand &cmd) {
    if (cmd.lastUpdate == 0) return false; // Uninitialized
    
    uint32_t timeoutMs = (cmd.timeout == 0) ? 100 : (cmd.timeout * 10); // Default 100ms
    return (millis() - cmd.lastUpdate) < timeoutMs;
}

// teensy3 manual control via CAN commands

void motorControlISR() {
    for (uint8_t i = 0; i < 4; i++) { // Control motors 1-4 only
        // Check if homing is in progress for this motor
        if (homingData[i].state == HOMING_IN_PROGRESS) {
            updateHoming(i);
            continue; // Skip normal command processing during homing
        }
        
        // Block all commands until homing is complete
        if (!isHomingComplete(i)) {
            motors.sendCurrent(i + 1, 0);
            startHoming(i);
            continue; // Skip command processing until homed
        }
        
        const MotorCommand &cmd = motorCommands[i];
        
        if (!isCommandValid(cmd)) {
            // Stop on timeout
            motors.sendCurrent(i + 1, 0);
            continue;
        }
        
        float value = convertToFloat(cmd.value);
        
        switch (cmd.mode) {
            case 0x00: // Disable
                motors.sendCurrent(i + 1, 0);
                break;
                
            case 0x01: // Speed control [rad/s]
                motorSpeedControl(i, value);
                break;
                
            case 0x02: // BoundedAngle control [rad]
                motorAngleControl(i, value, true);
                break;
                
            case 0x03: // UnboundedAngle control [rad]
                motorAngleControl(i, value, false);
                break;
                
            case 0xFF: // Special homing command (always allowed)
                if (!isHomingComplete(i)) {
                    startHoming(i);
                }
                break;
                
            default:
                motors.sendCurrent(i + 1, 0); // Stop on unknown mode
                break;
        }
    }
    
    motors.flush(); // Send all commands
}


// DIP switch pin definitions (for ID setting)
#define DIP_PIN1 12
#define DIP_PIN2 13

uint8_t readBoardId() {
    // Read 2-bit board ID (1-4) from DIP switches
    uint8_t id = 0;
    if (!digitalRead(DIP_PIN1)) id += 1;
    if (!digitalRead(DIP_PIN2)) id += 2;
    return id + 1; // 0-3 → 1-4
}

void setup() {
  Serial.begin(9600);
  Serial4.begin(115200);
  
  // DIP switch pin setup
  pinMode(DIP_PIN1, INPUT_PULLUP);
  pinMode(DIP_PIN2, INPUT_PULLUP);

  delay(100);
  // Read Board ID
  myBoardId = readBoardId();
  Serial.printf("Board ID: %d (teensy3)\n", myBoardId);  
  
  // Initialize homing system
  initializeHoming();
    startHoming(0); // Start homing for motor 1 by default
    startHoming(1); // Start homing for motor 2 by default
    startHoming(2); // Start homing for motor 3 by default

  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleCANMessage);
  
  // Start motor control timer (1kHz = 1000μs interval)
  motorControlTimer.begin(motorControlISR, 1000);
  motorControlTimer.priority(128);

  motors.setGearRatio(1, 125.0f);
  motors.setGearRatio(2, 100.0f);
  motors.setGearRatio(3, 28.0f);

  Serial.println("teensy3 (Motor Control with Homing) Ready");
  Serial.println("Send CAN command with mode 0xFF to start homing");
}

void loop() {
  // teensy3 responds to CAN commands only
  // Control executed by timer interrupt
  delay(10);
}