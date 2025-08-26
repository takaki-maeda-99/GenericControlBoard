#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "../../lib/DjiMotor.hpp"
#include "../../lib/PID.h"
#include "../../lib/CanMotorController.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan<CAN2> motors(can2, 3591.0f/187.0f);

// DIPスイッチピン定義 (ID設定用)
#define DIP_PIN1 12
#define DIP_PIN2 13

// odometryEncoderPIN
#define ODOMETRY_X_ENCODER_A_PIN 2
#define ODOMETRY_X_ENCODER_B_PIN 3
#define ODOMETRY_Y_ENCODER_A_PIN 4
#define ODOMETRY_Y_ENCODER_B_PIN 5

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

uint8_t myBoardId = 2;

// CANモーターコントローラー
CanMotorController motorController(2, motors, 4); // Board ID=2, 4モーター

// オドメトリエンコーダー変数
volatile int32_t encoderX_count = 0;
volatile int32_t encoderY_count = 0;
volatile uint32_t lastSpeedUpdate = 0;
volatile int32_t lastEncoderX_count = 0;
volatile int32_t lastEncoderY_count = 0;
float speedX = 0.0f;  // [m/s]
float speedY = 0.0f;  // [m/s]

// エンコーダー仕様
const float ENCODER_RESOLUTION = 100.0f;  // パルス/回転
const float WHEEL_DIAMETER = 0.038f;       // ホイール直径 [m] (38mm)
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;  // 円周 [m]
const uint32_t SPEED_UPDATE_INTERVAL = 10; // 速度更新間隔 [ms]


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

// オドメトリ速度送信フレーム（ID: 0x200）
struct OdometrySpeedFrame {
  float speedX;            // 4バイト
  float speedY;            // 4バイト
} __attribute__((packed));



struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};

// 14000 / 52 = 269.2307692307692
constexpr PidParam WheelSpeedPidParam{400, 10, 0, -14000, 14000, 1};

// 使い回すときに feed できるユーティリティ
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持
Pid WheelSpeedPid[4] = { makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam), makePID(WheelSpeedPidParam)};

// エンコーダー割り込み処理
void encoderX_ISR() {
    int stateA = digitalRead(ODOMETRY_X_ENCODER_A_PIN);
    int stateB = digitalRead(ODOMETRY_X_ENCODER_B_PIN);
    
    // A相の変化でカウント、B相の状態で方向判定
    if (stateA == stateB) {
        encoderX_count--;
    } else {
        encoderX_count++;
    }
}

void encoderY_ISR() {
    int stateA = digitalRead(ODOMETRY_Y_ENCODER_A_PIN);
    int stateB = digitalRead(ODOMETRY_Y_ENCODER_B_PIN);
    
    // A相の変化でカウント、B相の状態で方向判定
    if (stateA == stateB) {
        encoderY_count--;
    } else {
        encoderY_count++;
    }
}

// 速度計算関数
void updateOdometrySpeed() {
    uint32_t currentTime = millis();
    if (currentTime - lastSpeedUpdate >= SPEED_UPDATE_INTERVAL) {
        // 時間間隔を計算 [s]
        float deltaTime = (currentTime - lastSpeedUpdate) / 1000.0f;
        
        // エンコーダーカウントの差分を計算
        int32_t deltaX = encoderX_count - lastEncoderX_count;
        int32_t deltaY = encoderY_count - lastEncoderY_count;
        
        // 回転数に変換 [回転/s]
        float rpsX = deltaX / (ENCODER_RESOLUTION * deltaTime);
        float rpsY = deltaY / (ENCODER_RESOLUTION * deltaTime);
        
        // 線速度に変換 [m/s]
        speedX = rpsX * WHEEL_CIRCUMFERENCE;
        speedY = rpsY * WHEEL_CIRCUMFERENCE;
        
        // 前回値を更新
        lastEncoderX_count = encoderX_count;
        lastEncoderY_count = encoderY_count;
        lastSpeedUpdate = currentTime;
    }
}

// 速度取得関数
float getSpeedX() {
    return speedX;
}

float getSpeedY() {
    return speedY;
}

// エンコーダーカウント取得関数
int32_t getEncoderX() {
    return encoderX_count;
}

int32_t getEncoderY() {
    return encoderY_count;
}

// オドメトリ速度をCANで送信
void sendOdometrySpeedToCAN() {
    CAN_message_t msg;
    msg.id = 0x200;
    msg.len = 8;
    
    OdometrySpeedFrame frame;
    frame.speedX = speedX;
    frame.speedY = speedY;
    
    memcpy(msg.buf, &frame, sizeof(frame));
    can1.write(msg);
}

// ホイール速度制御関数
inline void wheelSpeedControl(size_t idx, float targetSpeed, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(idx + 1);
    
    int16_t speedCmd = WheelSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}


// CANメッセージハンドラー
static void handleMotorCommand(const CAN_message_t &msg) {
    motorController.handleCanMessage(msg);
}

IntervalTimer motorControlTimer;

// モーター制御ISR - ライブラリで処理
void motorControlISR() {
    motorController.executeCommands();
}




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
  
  // オドメトリエンコーダーピン設定
  pinMode(ODOMETRY_X_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_X_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_Y_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_Y_ENCODER_B_PIN, INPUT_PULLUP);
  
  // エンコーダー割り込み設定（立ち上がりのみ）
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_X_ENCODER_A_PIN), encoderX_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_Y_ENCODER_A_PIN), encoderY_ISR, RISING);

  delay(100);
// Board ID読み取り
  myBoardId = readBoardId();
  Serial.printf("Board ID: %d\n", myBoardId);  
  
  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleMotorCommand);
  
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
  // オドメトリ速度を更新
  updateOdometrySpeed();
  
  // オドメトリ速度をCANで送信
  sendOdometrySpeedToCAN();
  
  Serial.print("SpeedX: ");
  Serial.print(getSpeedX());
  Serial.print(" m/s, SpeedY: ");
  Serial.print(getSpeedY());
  Serial.println(" m/s");
  // teensy2はCANコマンドのみに応答
  // タイマー割り込みで制御実行
  delay(10);
}