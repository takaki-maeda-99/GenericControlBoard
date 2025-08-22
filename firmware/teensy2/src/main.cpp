#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "DjiMotor.hpp"
#include "PID.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan motors(can2, 3591.0f/187.0f);  // 36:1 ギア比で初期化 (ホイール用)

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

#define PI 3.14159265358979323846

inline void wheelSpeedControl(size_t idx, float targetSpeed) {
    const auto &fb = motors.feedback(idx + 1);
    
    int16_t speedCmd = WheelSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}

struct MotorCommand {
    uint8_t motorId;     // モーターID (1-8)
    uint8_t mode;        // 0x00=Disable, 0x01=Speed, 0x02=BoundedAngle, 0x03=UnboundedAngle
    uint8_t opt;         // 将来拡張/フラグ (現状0固定)
    int32_t value;       // 指令値 (rad/s または rad)
    uint8_t seq;         // シーケンス番号 (0-255巡回)
    uint8_t timeout;     // タイムアウト [10ms単位] (0=ボード既定)
    uint32_t lastUpdate; // 最終更新時刻 [ms]
    
    MotorCommand() : motorId(0), mode(0x00), opt(0), value(0), seq(0), timeout(0), lastUpdate(0) {}
} __attribute__((packed));

// 各モーターの最新コマンド
MotorCommand motorCommands[8];

bool parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd) {
    // CANメッセージIDから BoardId と MotorId を抽出
    uint8_t boardId = (msg.id >> 4) & 0x0F;  // 上位4bit
    uint8_t motorId = msg.id & 0x0F;         // 下位4bit
    
    // 自分宛てでない、またはモーターID範囲外
    if (boardId != myBoardId || motorId < 1 || motorId > 8) {
        return false;
    }
    
    // CANデータ長チェック
    if (msg.len < 8) {
        return false;
    }
    
    // CANメッセージから構造体に変換
    cmd.motorId = motorId;
    cmd.mode = msg.buf[0];
    cmd.opt = msg.buf[1];
    
    // int32値をリトルエンディアンで復元
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
    // int32からfloatへの変換 (固定小数点想定: 1000倍スケール)
    return (float)value / 1000.0f;
}

static void handleCANMessage(const CAN_message_t &msg) {
    // モーターコマンド処理のみ (0x02X)
    MotorCommand cmd;
    if (parseMotorCommand(msg, cmd)) {
        uint8_t idx = cmd.motorId - 1;  // 0-7インデックス
        
        // シーケンス番号チェック（重複コマンド拒否）
        if (cmd.seq == motorCommands[idx].seq && motorCommands[idx].lastUpdate > 0) {
            return; // 同じシーケンス番号は無視
        }
        
        // コマンド更新
        motorCommands[idx] = cmd;
        
        Serial.printf("Motor%d: mode=%02X, val=%.3f, seq=%d\n", 
                     cmd.motorId, cmd.mode, convertToFloat(cmd.value), cmd.seq);
    }
}

IntervalTimer motorControlTimer;

bool isCommandValid(const MotorCommand &cmd) {
    if (cmd.lastUpdate == 0) return false; // 未初期化
    
    uint32_t timeoutMs = (cmd.timeout == 0) ? 100 : (cmd.timeout * 10); // デフォルト100ms
    return (millis() - cmd.lastUpdate) < timeoutMs;
}

void motorControlISR() {
    for (uint8_t i = 0; i < 4; i++) { // モーター1-4のみ制御
        const MotorCommand &cmd = motorCommands[i];
        
        if (!isCommandValid(cmd)) {
            // タイムアウト時は停止
            motors.sendCurrent(i + 1, 0);
            continue;
        }
        
        float value = convertToFloat(cmd.value);
        
        switch (cmd.mode) {
            case 0x00: // Disable
                motors.sendCurrent(i + 1, 0);
                break;
                
            case 0x01: // Speed control [rad/s]
                wheelSpeedControl(i, value);
                break;
                
            case 0x02: // BoundedAngle control [rad] (ホイールでは未使用)
            case 0x03: // UnboundedAngle control [rad] (ホイールでは未使用)
                motors.sendCurrent(i + 1, 0); // ホイールでは角度制御しない
                break;
                
            default:
                motors.sendCurrent(i + 1, 0); // 不明なモードは停止
                break;
        }
    }
    
    motors.flush(); // 全コマンド送信
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