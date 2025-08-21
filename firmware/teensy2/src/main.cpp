#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

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

// 受信したコントローラデータ
ControllerFrame1 frame1Data = {0};
ControllerFrame2 frame2Data = {0};
bool dataReceived = false;

void setup() {
  Serial.begin(57600);
  
  // CAN初期化
  Can1.begin();
  Can1.setBaudRate(500000);
  
  Serial.println("Teensy 4.0 CAN受信開始");
  Serial.println("コントローラデータID: 0x100");
}

void readCANMessage() {
  CAN_message_t msg;
  
  if (Can1.read(msg)) {
    // フレーム1: state + mode + スティック（ID: 0x100）
    if (msg.id == 0x100 && msg.len == 6) {
      memcpy(&frame1Data, msg.buf, sizeof(ControllerFrame1));
      Serial.print("フレーム1受信: state=");
      Serial.print(frame1Data.state);
      Serial.print(" mode=");
      Serial.print(frame1Data.mode);
      Serial.print(" lx=");
      Serial.print(frame1Data.lx);
      Serial.print(" ly=");
      Serial.print(frame1Data.ly);
      Serial.print(" rx=");
      Serial.print(frame1Data.rx);
      Serial.print(" ry=");
      Serial.print(frame1Data.ry);
    }
    // フレーム2: ボタン + トリガー（ID: 0x101）
    else if (msg.id == 0x101 && msg.len == 4) {
      memcpy(&frame2Data, msg.buf, sizeof(ControllerFrame2));
      dataReceived = true;
      
      Serial.print("  フレーム2受信: buttons=0x");
      Serial.print(frame2Data.buttons, HEX);
      Serial.print(" l2=");
      Serial.print(frame2Data.l2);
      Serial.print(" r2=");
      Serial.println(frame2Data.r2);
    }
  }
}

void loop() {
  readCANMessage();
  
  // 受信データがあれば処理
  if (dataReceived) {
    // 両フレームのデータを使って処理
    // frame1Data: state, mode, lx, ly, rx, ry
    // frame2Data: buttons, l2, r2
    
    dataReceived = false; // フラグをリセット
  }
  
  delay(1); // CPUの負荷を軽減
}