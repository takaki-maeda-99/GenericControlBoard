#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include "../../lib/DjiMotor.hpp"
#include "../../lib/PID.h"
#include "../../lib/CanMotorController.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan<CAN2> motors(can2, 129.5f);

uint8_t myBoardId = 3;

// サーボモーター (motorId=9用)
Servo servo;
int servo_angle = 0;
#define SERVO_PIN 9

// CANモーターコントローラー
CanMotorController motorController(myBoardId, 4); // Board ID=3, 4モーター

float homeAngles[3] = {-PI-0.2f, PI, PI}; // ホーミング後の角度設定

struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};

constexpr PidParam MotorAnglePidParam[4] = {
    {4000, 0, 0, -14000, 14000, 10}, // Motor 1
    {4000, 0, 0, -14000, 14000, 10}, // Motor 2
    {2000, 0, 0, -14000, 14000, 10}, // Motor 3
    {0, 0, 0, -14000, 14000, 10}  // Motor 4
};
constexpr PidParam MotorSpeedPidParam[4] = {
    {1, 0, 0, -10000, 10000, 1}, // Motor 1
    {1, 0, 0, -10000, 10000, 1}, // Motor 2
    {1, 0, 0, -8000, 8000, 1}, // Motor 3
    {0, 0, 0, -10000, 10000, 1}  // Motor 4
};

// 使い回すときに feed できるユーティリティ
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持
Pid MotorAnglePid[4] = { makePID(MotorAnglePidParam[0]), makePID(MotorAnglePidParam[1]), makePID(MotorAnglePidParam[2]), makePID(MotorAnglePidParam[3])};
Pid MotorSpeedPid[4] = { makePID(MotorSpeedPidParam[0]), makePID(MotorSpeedPidParam[1]), makePID(MotorSpeedPidParam[2]), makePID(MotorSpeedPidParam[3])};

// 過負荷検知用変数
uint32_t overloadCounter[4] = {0, 0, 0, 0};
bool motorStopped[4] = {false, false, false, false};

inline void motorSpeedControl(size_t motorIdx, float targetSpeed, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(motorIdx);
    
    int16_t speedCmd = MotorSpeedPid[motorIdx-1].compute(fb.speedRaw, targetSpeed);
    motors.sendCurrent(motorIdx, speedCmd);
}

inline void motorAngleControl(size_t motorIdx, float targetAngle, bool bounded, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(motorIdx);
    const float fbAngle = fb.getAngleRadians();
    const float fbSpeed = fb.speedRaw;

    int16_t angleCmd  = MotorAnglePid[motorIdx-1].compute(fbAngle, targetAngle);
    int16_t speedCmd  = MotorSpeedPid[motorIdx-1].compute(fbSpeed, angleCmd);

    motors.sendCurrent(motorIdx, speedCmd);
    
}


// CANメッセージハンドラー
static void handleMotorCommand(const CAN_message_t &msg) {
    motorController.handleCanMessage(msg);
}

volatile bool tick;

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

void homing(){
    // ホーミング中はモーターコントロール割込みを停止
    tick = false;

    int homingTimeout = 3000;  // 3秒のタイムアウト
    unsigned long startTime = millis();

    servo_angle = 90; // サーボを中央に

    while(millis() - startTime < homingTimeout){
        motorSpeedControl(1, -2500, motors);
        motorSpeedControl(2, 3000, motors);
        motorSpeedControl(3, 2000, motors);
        motors.flush();
        delay(1);
    }
    motors.sendCurrent(1, 0);
    motors.sendCurrent(2, 0);
    motors.sendCurrent(3, 0);

    for (int i=0; i<3; i++) motors.resetAngle(i+1, homeAngles[i]);
    // ホーミング完了後、モーターコントロール割り込みを再開

    tick = true;
}

void setup() {
  Serial.begin(9600);
  Serial4.begin(115200);
  
  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleMotorCommand);
  
  // モーターコントローラー初期化

  motors.setGearRatio(1, 126.0f);
  motors.setGearRatio(2, 95.0f);
  motors.setGearRatio(3, 29.0f);

  // モーター制御コールバック設定
  motorController.setControlCallback([](uint8_t idx, const MotorCommand &cmd) {
        Serial.printf("Motor %d: mode=0x%02X, value=%.3f\n", idx+1, cmd.mode, CanMotorController::convertToFloat(cmd.value));
        float value = CanMotorController::convertToFloat(cmd.value);
        if(tick == true){
            switch (cmd.mode) {
                case 0x00: // Disable
                    motorAngleControl(idx+1, value, true, motors);
                    break;
                    
                case 0x01: // Speed control [rad/s]
                    motorSpeedControl(idx+1, value, motors);
                    break;
                    
                case 0x02: // BoundedAngle control [rad]
                    motorAngleControl(idx+1, value, true, motors);
                    if (idx == 3) {
                        servo_angle = int(value);
                    }
                    break;
                case 0x03: // UnboundedAngle control [rad]
                    motorAngleControl(idx+1, value, false, motors);
                    break;
                case 0xFF: // Homing command
                    homing();
                    break;
                default:
                    motors.sendCurrent(idx+1, 0); // Stop on unknown mode
                    break;
            }
        }
    });

  // サーボ初期化
  servo.attach(SERVO_PIN,580,2500);

  servo.write(90);
  homing();

    tick = true;

  Serial.println("teensy3 (Motor Control with Homing) Ready");
}


void loop() {
    servo.write(servo_angle);
    const auto &fb1 = motors.feedback(1);
    const auto &fb2 = motors.feedback(2);
    const auto &fb3 = motors.feedback(3);

    // Serial.print(servo.read());
    // Serial.print(", ");
    motors.flush();
  delay(1);
}