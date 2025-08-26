#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "../../lib/DjiMotor.hpp"
#include "../../lib/PID.h"
#include "../../lib/HomingController.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan motors(can2, 129.5f);

// コントローラ入力データ
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

uint8_t myBoardId = 3;

const float homeAngles[4] = {-PI+0.05,-PI+0.05,-PI+0.05,-PI};

// 共通ホーミングコントローラー
HomingController homingController(4, homeAngles);

// teensy3固有のホーミングパラメータ
const int16_t HOMING_CURRENT = -1500;        // mA (negative direction)
const uint32_t HOMING_TIME = 2000;          // ms (time to apply current)

// CANフレーム構造体
struct ControllerFrame1 {
  uint8_t state;           // 1バイト
  uint8_t mode;            // 1バイト
  int8_t lx, ly;          // 2バイト
  int8_t rx, ry;          // 2バイト
} __attribute__((packed));

struct ControllerFrame2 {
  uint16_t buttons;        // 2バイト
  uint8_t l2, r2;         // 2バイト（フル精度）
} __attribute__((packed));


struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};


constexpr PidParam MotorAnglePidParam{20, 0, 0, -30, 30, 10};
constexpr PidParam MotorSpeedPidParam{500, 0, 0, -8000, 8000, 1};

// 使い回すときに feed できるユーティリティ
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持
Pid MotorAnglePid[4] = { makePID(MotorAnglePidParam), makePID(MotorAnglePidParam), makePID(MotorAnglePidParam), makePID(MotorAnglePidParam)};
Pid MotorSpeedPid[4] = { makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam), makePID(MotorSpeedPidParam)};

// #define PI 3.14159265358979323846  // Already defined by Arduino

inline void motorSpeedControl(size_t idx, float targetSpeed) {
    const auto &fb = motors.feedback(idx + 1);
    
    int16_t speedCmd = MotorSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}

inline void motorAngleControl(size_t idx, float targetAngle, bool bounded) {
    const auto &fb = motors.feedback(idx + 1);
    const float fbAngle = fb.getAngleRadiansWrapped();
    const float fbSpeed = fb.getSpeedRadiansPerSec();
    
    if(bounded) {
        // 目標角度を制限
        targetAngle = constrain(targetAngle, -PI, PI);
    }
    else {
        // 無限回転
        // 半回転より大きい場合は、目標角度を調整
        if(targetAngle - fbAngle > PI) {
            targetAngle -= DjiConstants::PI_2;
        } else if(targetAngle - fbAngle < -PI) {
            targetAngle += DjiConstants::PI_2;
        }
    }
    
    int16_t angleCmd  = MotorAnglePid[idx].compute(fbAngle, targetAngle);
    int16_t speedCmd  = MotorSpeedPid[idx].compute(fbSpeed, angleCmd);
    
    motors.sendCurrent(idx + 1, speedCmd);
}

void initializeHoming() {
    homingController.initialize();
    
    // teensy3固有のホーミング手順設定（時間ベース）
    homingController.setProcedureCallback([](uint8_t idx, DjiMotorCan<CAN2> &motors) {
        static uint32_t startTimes[4] = {0};
        
        // 初回実行時に開始時刻を記録
        if (startTimes[idx] == 0) {
            // モーターフィードバックが利用可能かチェック
            const auto &fb = motors.feedback(idx + 1);
            if (fb.isTimeout) {
                Serial.printf("Motor %d: No feedback - cannot start homing\n", idx + 1);
                return true; // エラーとして完了扱い
            }
            
            startTimes[idx] = millis();
            Serial.printf("Starting homing for motor %d (feedback confirmed)\n", idx + 1);
        }
        
        uint32_t currentTime = millis();
        
        // ホーミング時間チェック
        if (currentTime - startTimes[idx] >= HOMING_TIME) {
            // ホーミング完了
            startTimes[idx] = 0; // リセット
            return true;
        }
        
        // ホーミング用電流印加
        motors.sendCurrent(idx + 1, HOMING_CURRENT);
        return false; // 継続中
    });
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

// CANメッセージハンドラー
static void handleMotorCommand(const CAN_message_t &msg) {
    // Motor command processing only (0x03X)
    MotorCommand cmd;
    if (parseMotorCommand(msg, cmd)) {
        uint8_t idx = cmd.motorId - 1;  // 0-7 index
        
        // Allow homing command (0xFF) even if not homed
        if (cmd.mode != 0xFF && !homingController.isHomingComplete(idx)) {
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
    // まずホーミング処理を実行
    homingController.update(motors);
    
    for (uint8_t i = 0; i < 4; i++) { // Control motors 1-4 only
        // ホーミング中はコマンド処理をスキップ
        if (homingController.getState(i) == HOMING_IN_PROGRESS) {
            continue;
        }
        
        // ホーミング未完了の場合は停止
        if (!homingController.isHomingComplete(i)) {
            motors.sendCurrent(i + 1, 0);
            continue;
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
                homingController.handleHomingCommand(i);
                break;
                
            default:
                motors.sendCurrent(i + 1, 0); // Stop on unknown mode
                break;
        }
    }
    
    motors.flush(); // Send all commands
}


// DIPスイッチピン定義 (ID設定用)
#define DIP_PIN1 12
#define DIP_PIN2 13

uint8_t readBoardId() {
    // DIPスイッチから2bitでboard ID (1-4)を読み取り
    uint8_t id = 0;
    if (!digitalRead(DIP_PIN1)) id += 1;
    if (!digitalRead(DIP_PIN2)) id += 2;
    return id + 1; // 0-3 → 1-4
}

void setup() {
  Serial.begin(9600);
  Serial4.begin(115200);
  
  // DIPスイッチピン設定
  pinMode(DIP_PIN1, INPUT_PULLUP);
  pinMode(DIP_PIN2, INPUT_PULLUP);

  delay(100);
// Board ID読み取り
  myBoardId = readBoardId();
  Serial.printf("Board ID: %d\n", myBoardId);  
  
  // Initialize homing system
  initializeHoming();
  homingController.startAllHoming(); // Start homing for all motors

  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleMotorCommand);
  
// モーター制御タイマー開始 (1kHz = 1000μs間隔)
  motorControlTimer.begin(motorControlISR, 1000);
  motorControlTimer.priority(128);

  motors.setGearRatio(1, 125.0f);
  motors.setGearRatio(2, 100.0f);
  motors.setGearRatio(3, 28.0f);

  Serial.println("teensy3 (Motor Control with Homing) Ready");
}


void loop() {
  // teensy3はCANコマンドのみに応答
  // タイマー割り込みで制御実行
  delay(10);
}