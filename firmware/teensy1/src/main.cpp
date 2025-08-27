#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "../../lib/DjiMotor.hpp"
#include "../../lib/PID.h"
#include "../../lib/CanMotorController.h"

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
DjiMotorCan<CAN2> motors(can2, 72.0f);  // 72:1 ギア比で初期化


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

uint8_t myBoardId = 1;

// CANモーターコントローラー
CanMotorController motorController(1, motors, 4); // Board ID=1, 4モーター

const float homeAngles[4] = {PI/2.0f, 0.0f, -PI/2.0f, PI};

// コントローラーデータ読み取り関数
bool SerialRead(){
    if(Serial4.available()) {
    String data = Serial4.readStringUntil('\n');

    int values[8];
    int index = 0;
    // データをコピーしてstrtokに渡す (const char*の警告を回避)
    char buffer[data.length() + 1];
    data.toCharArray(buffer, sizeof(buffer));
    char* token = strtok(buffer, ",");
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

    return true;
    }
    return false;
}

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

// コントローラーデータをCANバスに送信
void sendControllerDataToCAN() {
    CAN_message_t msg;
    
    // フレーム1: state + スティック（ID: 0x100）
    msg.id = 0x100;
    msg.len = 6;
    
    ControllerFrame1 frame1;
    frame1.state = (uint8_t)state;
    frame1.mode = 0; // 固定値
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


// ステアリング角度制御関数
inline void angleControl(size_t idx, float targetAngle, bool bounded, DjiMotorCan<CAN2> &motors) {
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

// ステアリング速度制御関数
inline void speedControl(size_t idx, float targetSpeed, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(idx + 1);

    int16_t speedCmd = SpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors.sendCurrent(idx + 1, speedCmd);
}


bool readLimitSwitch1(uint8_t steerIndex) {
    uint8_t lsPin1;
    
    switch(steerIndex) {
        case 0: lsPin1 = LSPIN11; break;
        case 1: lsPin1 = LSPIN21; break;
        case 2: lsPin1 = LSPIN31; break;
        case 3: lsPin1 = LSPIN41; break;
        default: return false;
    }
    
    return digitalRead(lsPin1);
}

bool readLimitSwitch2(uint8_t steerIndex) {
    uint8_t lsPin2;
    
    switch(steerIndex) {
        case 0: lsPin2 = LSPIN12; break;
        case 1: lsPin2 = LSPIN22; break;
        case 2: lsPin2 = LSPIN32; break;
        case 3: lsPin2 = LSPIN42; break;
        default: return false;
    }
    
    return digitalRead(lsPin2);
}

// CANメッセージハンドラー
static void handleMotorCommand(const CAN_message_t &msg) {
    motorController.handleCanMessage(msg);
}

IntervalTimer motorControlTimer;


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

void homing(){
    // ホーミング中はモーターコントロール割込みを停止
    motorControlTimer.end();
    
    bool homingComplete[4] = {false};
    while(!homingComplete[0] || !homingComplete[1] || !homingComplete[2] || !homingComplete[3]) {
    // 無限ループ
        for(int i=0; i<4; i++) {
            bool ls1 = readLimitSwitch1(i);
            bool ls2 = readLimitSwitch2(i);

            if((ls1 && ls2) || homingComplete[i] ) {
                // 両方のスイッチが押されたらホーミング完了
                motors.resetAngle(i+1, homeAngles[i]);
                speedControl(i, 0, motors);
                homingComplete[i] = true;
            }
            else if(ls1) {
                // スイッチ1が押されたらホーミング中
                speedControl(i, 3, motors);
            }
            else{
                // スイッチ2が押されたらホーミング中
                speedControl(i, 20, motors);
            }
        }
        motors.flush();
        Serial.print(homingComplete[0]);
        Serial.print(homingComplete[1]);
        Serial.print(homingComplete[2]);
        Serial.println(homingComplete[3]);
        delay(1);
    }
    
    // ホーミング完了後、モーターコントロール割込みを再開
    motorControlTimer.begin(motorControlISR, 1000);
    motorControlTimer.priority(128);
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
  
  // モーターコントローラー初期化
  motorController.setDebugOutput(false);
  
  // ステアリング制御コールバック設定
  motorController.setControlCallback([](uint8_t idx, const MotorCommand &cmd, DjiMotorCan<CAN2> &motors) {
      float value = CanMotorController::convertToFloat(cmd.value);
      
      switch (cmd.mode) {
          case 0x00: // Disable
              motors.sendCurrent(idx + 1, 0);
              break;
              
          case 0x01: // Speed control [rad/s]
              speedControl(idx, value, motors);
              break;
              
          case 0x02: // BoundedAngle control [rad]
              angleControl(idx, value, true, motors);
              break;
              
          case 0x03: // UnboundedAngle control [rad]
              angleControl(idx, value, false, motors);
              break;
              
          case 0xFF: // Homing command
              homing();
              break;
              
          default:
              motors.sendCurrent(idx + 1, 0); // 不明なモードは停止
              break;
      }
  });

  pinMode(LSPIN11, INPUT_PULLUP);
  pinMode(LSPIN12, INPUT_PULLUP);
  pinMode(LSPIN21, INPUT_PULLUP);
  pinMode(LSPIN22, INPUT_PULLUP);
  pinMode(LSPIN31, INPUT_PULLUP);
  pinMode(LSPIN32, INPUT_PULLUP);
  pinMode(LSPIN41, INPUT_PULLUP);
  pinMode(LSPIN42, INPUT_PULLUP);

// モーター制御タイマー開始 (1kHz = 1000μs間隔)
  motorControlTimer.begin(motorControlISR, 1000);
  motorControlTimer.priority(128);

  homing();
  motorController.setDebugOutput(true);

}


void loop() {
  // コントローラーデータをCANバスに転送
  if(SerialRead()) {
    sendControllerDataToCAN();
    
    // triangleボタンでホーミング再実行（削除）
    if(button.triangle) {
        Serial.println("Homing restart requested");
    }
  }
  
  delay(10);
}