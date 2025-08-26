#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "../../lib/DjiMotor.hpp"
#include "../../lib/PID.h"
#include "../../lib/CanMotorController.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan<CAN2> motors(can2, 3591.0f/187.0f);  // 36:1 ギア比で初期化 (ホイール用)

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

uint8_t myBoardId = 2; // teensy2なのでBoard ID = 2

// CANモーターコントローラー
CanMotorController motorController(2, motors, 4); // Board ID=2, 4モーター


// フレーム1: state + mode + スティック（6バイト）
struct ControllerFrame1 {
  uint8_t state;           // 1バイト
  uint8_t mode;            // 1バイト
  int8_t lx, ly;          // 2バイト
  int8_t rx, ry;          // 2バイト
} __attribute__((packed));

// フレーム2: ボタン + トリガー（4バイト）
struct ControllerFrame2 {
  uint16_t buttons;        // 2バイト
  uint8_t l2, r2;         // 2バイト（フル精度）
} __attribute__((packed));


// teensy2はCANコマンドのみに応答（モード管理なし）

// CANからコントローラデータを受信
ControllerFrame1 controllerFrame1 = {0};
ControllerFrame2 controllerFrame2 = {0};
bool controllerDataReceived = false;

// teensy2はコントローラデータを処理しない

// teensy2はコントローラデータを送信しない（受信のみ）

struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};

// ホイール用PIDパラメータ
constexpr PidParam WheelSpeedPidParam{800, 10, 0, -14000, 14000, 1};

// 使い回すときに feed できるユーティリティ
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持 (ホイール4個)
Pid WheelSpeedPid[4] = { makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam)};

// #define PI 3.14159265358979323846  // Already defined by Arduino

// ホイール速度制御関数
inline void wheelSpeedControl(size_t idx, float targetSpeed, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(idx + 1);
    
    int16_t speedCmd = WheelSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}


// CANメッセージハンドラー
static void handleCANMessage(const CAN_message_t &msg) {
    motorController.handleCanMessage(msg);
}

IntervalTimer motorControlTimer;

// モーター制御ISR - ライブラリで処理
void motorControlISR() {
    motorController.executeCommands();
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
  Serial.printf("Board ID: %d (teensy2)\n", myBoardId);  
  
  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleCANMessage);
  
  // モーターコントローラー初期化
  motorController.setDebugOutput(false);
  
  // ホイール制御コールバック設定
  motorController.setControlCallback([](uint8_t idx, const MotorCommand &cmd, DjiMotorCan<CAN2> &motors) {
      float value = CanMotorController::convertToFloat(cmd.value);
      
      switch (cmd.mode) {
          case 0x00: // Disable
              motors.sendCurrent(idx + 1, 0);
              break;
              
          case 0x01: // Speed control [rad/s]
              wheelSpeedControl(idx, value, motors);
              break;
              
          case 0x02: // BoundedAngle control [rad] (ホイールでは未使用)
          case 0x03: // UnboundedAngle control [rad] (ホイールでは未使用)
              motors.sendCurrent(idx + 1, 0); // ホイールでは角度制御しない
              break;
              
          default:
              motors.sendCurrent(idx + 1, 0); // 不明なモードは停止
              break;
      }
  });
  
  // モーター制御タイマー開始 (1kHz = 1000μs間隔)
  motorControlTimer.begin(motorControlISR, 1000);
  motorControlTimer.priority(128);

  Serial.println("teensy2 (Wheel Control) Ready");
}

void loop() {
  // teensy2はCANコマンドのみに応答
  // タイマー割り込みで制御実行
  delay(10);
}