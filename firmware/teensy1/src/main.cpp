#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "DjiMotor.hpp"
#include "PID.h"

#define LSPIN11 2
#define LSPIN12 3
#define LSPIN21 4
#define LSPIN22 5
#define LSPIN31 6
#define LSPIN32 7
#define LSPIN41 8
#define LSPIN42 9

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan motors(can2, 72.0f);  // 72:1 ギア比で初期化

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

uint8_t myBoardId = 1;


enum systemMode{
  MANUAL = 0,
  AUTO = 1,
  HOMING = 2
} mode = AUTO;

enum homingState{
  NOT_HOMED = 0,
  HOMING_IN_PROGRESS = 1,
  HOMED = 2
};

struct SteerHoming {
  homingState state;
  bool completed;
  float homeAngle;
} steerHoming[4];

const float homeAngles[4] = {PI/2.0f, 0.0f, -PI/2.0f, PI};


bool SerialRead(){
    if(Serial4.available()) {
    String data = Serial4.readStringUntil('\n');

    int values[8];
    int index = 0;
    char* token = strtok(data.c_str(), ",");
    while(token && index < 8) {
        values[index++] = atoi(token);
        token = strtok(NULL, ",");
    }
    
    // Update global variables
    state = values[0];
    buttons = values[1];
    lx = values[2];
    ly = values[3];
    rx = values[4];
    ry = values[5];
    l2 = values[6];
    r2 = values[7];

    button.square   = (values[1] & (1 << 0))!= 0;
    button.cross    = (values[1] & (1 << 1))!= 0;
    button.circle   = (values[1] & (1 << 2))!= 0;
    button.triangle = (values[1] & (1 << 3))!= 0;
    button.l1       = (values[1] & (1 << 4))!= 0;
    button.r1       = (values[1] & (1 << 5))!= 0;
    button.share    = (values[1] & (1 << 8))!= 0;
    button.options  = (values[1] & (1 << 9))!= 0;


    Serial.print("left_x: ");
    Serial.print(lx);
    Serial.print(", left_y: ");
    Serial.print(ly);
    Serial.print(", right_x: ");
    Serial.print(rx);
    Serial.print(", right_y: ");
    Serial.print(ry);
    Serial.print(", l2: ");
    Serial.print(l2);
    Serial.print(", r2: ");
    Serial.print(r2);
    Serial.print(", buttons: ");
    Serial.println(buttons, BIN);

    return true;
    }
    return false;
}

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

void sendControllerDataToCAN() {
    CAN_message_t msg;
    
    // フレーム1: state + mode + スティック（ID: 0x100）
    msg.id = 0x100;
    msg.len = 6;
    
    ControllerFrame1 frame1;
    frame1.state = (uint8_t)state;
    frame1.mode = (uint8_t)mode;
    frame1.lx = (int8_t)lx;
    frame1.ly = (int8_t)ly;
    frame1.rx = (int8_t)rx;
    frame1.ry = (int8_t)ry;
    
    memcpy(msg.buf, &frame1, sizeof(frame1));
    can1.write(msg);
    
    // フレーム2: ボタン + トリガー（ID: 0x101）
    msg.id = 0x101;
    msg.len = 4;
    
    ControllerFrame2 frame2;
    frame2.buttons = (uint16_t)buttons;
    frame2.l2 = (uint8_t)l2;
    frame2.r2 = (uint8_t)r2;
    
    memcpy(msg.buf, &frame2, sizeof(frame2));
    can1.write(msg);
}

struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};


constexpr PidParam AnglePidParam{30, 0, 0, -22.0, 22.0, 10};
constexpr PidParam SpeedPidParam{1500, 0, 0, -8000, 8000, 1};

// 使い回すときに feed できるユーティリティ
inline Pid makePID(const PidParam &p) {
    return Pid(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持
Pid AnglePid[4] = { makePID(AnglePidParam), makePID(AnglePidParam), makePID(AnglePidParam), makePID(AnglePidParam)};
Pid SpeedPid[4] = { makePID(SpeedPidParam), makePID(SpeedPidParam), makePID(SpeedPidParam), makePID(SpeedPidParam)};

#define PI 3.14159265358979323846

inline void angleControl(size_t idx, float targetAngle, bool bounded) {
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

    int16_t angleCmd  = AnglePid[idx].compute(fbAngle, targetAngle);
    int16_t speedCmd  = SpeedPid[idx].compute(fbSpeed, angleCmd);

    motors.sendCurrent(idx + 1, speedCmd);
}

inline void speedControl(size_t idx, float targetSpeed) {
    const auto &fb = motors.feedback(idx + 1);

    int16_t speedCmd = SpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
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

bool readLimitSwitches(uint8_t steerIndex) {
    uint8_t lsPin1, lsPin2;
    
    switch(steerIndex) {
        case 0: lsPin1 = LSPIN11; lsPin2 = LSPIN12; break;
        case 1: lsPin1 = LSPIN21; lsPin2 = LSPIN22; break;
        case 2: lsPin1 = LSPIN31; lsPin2 = LSPIN32; break;
        case 3: lsPin1 = LSPIN41; lsPin2 = LSPIN42; break;
        default: return false;
    }
    
    return digitalRead(lsPin1) && digitalRead(lsPin2);
}

static void handleMotorCommand(const CAN_message_t &msg) {
    MotorCommand cmd;
    if (parseMotorCommand(msg, cmd)) {
        uint8_t idx = cmd.motorId - 1;  // 0-7インデックス
        
        // シーケンス番号チェック（重複コマンド拒否）
        if (cmd.seq == motorCommands[idx].seq && motorCommands[idx].lastUpdate > 0) {
            return; // 同じシーケンス番号は無視
        }
        
        // コマンド更新
        motorCommands[idx] = cmd;
    }
}

IntervalTimer motorControlTimer;

bool isCommandValid(const MotorCommand &cmd) {
    if (cmd.lastUpdate == 0) return false; // 未初期化
    
    uint32_t timeoutMs = (cmd.timeout == 0) ? 100 : (cmd.timeout * 10); // デフォルト100ms
    return (millis() - cmd.lastUpdate) < timeoutMs;
}

float convertToFloat(int32_t value) {
    // int32からfloatへの変換 (固定小数点想定: 1000倍スケール)
    return (float)value / 1000.0f;
}

void initializeHoming() {
    for (uint8_t i = 0; i < 4; i++) {
        steerHoming[i].state = NOT_HOMED;
        steerHoming[i].completed = false;
        steerHoming[i].homeAngle = homeAngles[i];
    }
}

void homingControl() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (now - lastUpdate < 10) return;
    lastUpdate = now;
    
    bool allHomed = true;
    
    for (uint8_t i = 0; i < 4; i++) {
        MotorCommand &cmd = motorCommands[i];
        
        switch (steerHoming[i].state) {
            case NOT_HOMED:
                if (readLimitSwitches(i)) {
                    steerHoming[i].state = HOMED;
                } else {
                    steerHoming[i].state = HOMING_IN_PROGRESS;
                }
                allHomed = false;
                break;
                
            case HOMING_IN_PROGRESS:
                if (readLimitSwitches(i)) {
                    motors.resetAngle(i + 1, steerHoming[i].homeAngle);
                    steerHoming[i].state = HOMED;
                    steerHoming[i].completed = true;
                    cmd.motorId = i + 1;
                    cmd.mode = 0x03; // UnboundedAngle
                    cmd.opt = 0;
                    cmd.value = 0;
                    cmd.seq = 0xFE;
                    cmd.timeout = 10;
                    cmd.lastUpdate = now;
                } else {
                    cmd.motorId = i + 1;
                    cmd.mode = 0x01; // Speed control
                    cmd.opt = 0;
                    cmd.value = (int32_t)(10.0f * 1000.0f); // 10 rad/s
                    cmd.seq = 0xFE;
                    cmd.timeout = 10;
                    cmd.lastUpdate = now;
                }
                allHomed = false;
                break;
                
            case HOMED:
                cmd.motorId = i + 1;
                cmd.mode = 0x03; // UnboundedAngle
                cmd.opt = 0;
                cmd.value = 0;
                cmd.seq = 0xFE;
                cmd.timeout = 10;
                cmd.lastUpdate = now;
                break;
        }
    }
    
    if (allHomed) {
        mode = AUTO;
    }
}

void sendCAN(uint8_t boardId, uint8_t motorId, uint8_t mode, float value, uint8_t timeout = 10) {
    CAN_message_t msg;
    msg.id = (boardId << 4) | motorId; // BoardId + MotorId
    msg.len = 8;
    
    static uint8_t seq = 0;
    int32_t intValue = (int32_t)(value * 1000.0f); // 1000倍スケール
    
    msg.buf[0] = mode;    // Mode (0x01=Speed, 0x02=BoundedAngle, etc.)
    msg.buf[1] = 0x00;    // OPT
    msg.buf[2] = intValue & 0xFF;
    msg.buf[3] = (intValue >> 8) & 0xFF;
    msg.buf[4] = (intValue >> 16) & 0xFF;
    msg.buf[5] = (intValue >> 24) & 0xFF;
    msg.buf[6] = seq++;
    msg.buf[7] = timeout; // Timeout [10ms units]
    
    can1.write(msg);
}

void manualControl() {
    // マニュアルモード時のモーターコマンド直接制御
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // 10ms間隔で更新
    if (now - lastUpdate < 10) return;
    lastUpdate = now;
    
    const float half_length = 0.25f;   // 単位[m]（例）
    const float half_width  = 0.25f;   // 単位[m]（例） 

    // ホイール・ステアリング共通の計算
    float wheelX[4] = { +half_length, -half_length, -half_length, +half_length };
    float wheelY[4] = { +half_width,  +half_width,  -half_width,  -half_width  };

    // スティック値を正規化 (-127~127 → -1.0~1.0)
    float vx = lx / 127.0f;    // 前後方向
    float vy = ly / 127.0f;    // 左右方向  
    float omega = -4 * rx / 127.0f; // 旋回

    for (uint8_t i = 0; i < 4; i++) {
        float vx_i = vx - omega * wheelY[i];
        float vy_i = vy + omega * wheelX[i];
        
        // ステアリング角度計算
        float steering_input = atan2(vx_i, vy_i);
        
        // ホイール速度計算 (ベクトルの内積で方向決定)
        float wheel_speed = sqrt(vx_i * vx_i + vy_i * vy_i); // ホイール速度の大きさ


        // ステアリング制御 (teensy1のモーター1-4)
        MotorCommand &cmd = motorCommands[i];
        cmd.motorId = i + 1;
        cmd.mode = 0x03; // UnboundedAngle
        cmd.opt = 0;
        cmd.value = (int32_t)(steering_input * 1000.0f); // 1000倍スケール
        cmd.seq = 0xFF; // マニュアルモード識別用
        cmd.timeout = 10; // 100ms
        cmd.lastUpdate = now;
        
        // ホイール制御 (teensy2へのCANコマンド送信)
        sendCAN(2, i + 1, 0x01, wheel_speed * 40.0f); // Board2, Speed control
    }
    
    // アーム制御 (teensy3へのCANコマンド送信)
    
    // circleボタンでアームホーミング開始
    if (button.circle) {
        sendCAN(3, 1, 0xFF, 0); // Motor ID 1 homing
        sendCAN(3, 2, 0xFF, 0); // Motor ID 2 homing  
        sendCAN(3, 3, 0xFF, 0); // Motor ID 3 homing
    } else {
        // 通常のアーム制御
        // ry → Motor ID 1 (bounded angle control)
        float ry_angle = (ry / 128.0f)*30; // -π ~ +π rad
        sendCAN(3, 1, 0x01, ry_angle);
        
        // l2 → Motor ID 2 (bounded angle control)
        // float l2_angle = (l2-128.0f)* PI / 130.0f; // -π ~ +π rad
        // sendCAN(3, 2, 0x02, l2_angle);
        
        // r2 → Motor ID 3 (bounded angle control)
        float r2_angle = (r2-128.0f)* PI / 130.0f; // -π ~ +π rad
        sendCAN(3, 3, 0x02, r2_angle);
    }
    
    // l1/r1 → Motor ID 9 (servo) - bounded angle control
    static float servo_angle = 0.0f;
    if (button.l1) servo_angle = -PI/2; // -90 degrees
    if (button.r1) servo_angle = +PI/2; // +90 degrees
    sendCAN(3, 9, 0x02, servo_angle);
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
                speedControl(i, value);
                break;
                
            case 0x02: // BoundedAngle control [rad]
                angleControl(i, value, true);
                break;
                
            case 0x03: // UnboundedAngle control [rad]
                angleControl(i, value, false);
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
  Serial.printf("Board ID: %d\n", myBoardId);  
  
  can1.begin();
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(handleMotorCommand);
  
  // モーター制御タイマー開始 (1kHz = 1000μs間隔)
  motorControlTimer.begin(motorControlISR, 1000);
  motorControlTimer.priority(128);

  pinMode(LSPIN11, INPUT_PULLUP);
  pinMode(LSPIN12, INPUT_PULLUP);
  pinMode(LSPIN21, INPUT_PULLUP);
  pinMode(LSPIN22, INPUT_PULLUP);
  pinMode(LSPIN31, INPUT_PULLUP);
  pinMode(LSPIN32, INPUT_PULLUP);
  pinMode(LSPIN41, INPUT_PULLUP);
  pinMode(LSPIN42, INPUT_PULLUP);

  initializeHoming();
  mode = HOMING;
}


void loop() {
  if(SerialRead()) {
    sendControllerDataToCAN();
    if(button.options) mode = MANUAL;
    if(button.share) mode = AUTO;
    if(button.triangle) {
        initializeHoming();
        mode = HOMING;
    }
  }
  switch (mode) {
    case MANUAL:
      manualControl();
      break;
    case AUTO:
      // Auto mode logic here
      break;
    case HOMING:
      homingControl();
      break;
  }
  delay(10);
}