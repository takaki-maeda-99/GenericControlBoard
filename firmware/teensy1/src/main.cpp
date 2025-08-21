#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

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

    Serial.print("Received: ");
    Serial.print("state: " + String(state) + ", ");
    Serial.print("buttons: " + String(buttons) + ", ");
    Serial.print("lx: " + String(lx) + ", ");
    Serial.print("ly: " + String(ly) + ", ");
    Serial.print("rx: " + String(rx) + ", ");
    Serial.print("ry: " + String(ry) + ", ");
    Serial.print("l2: " + String(l2) + ", ");
    Serial.print("r2: " + String(r2));
    // Serial.println();

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
  Can1.write(msg);
  
  // フレーム2: ボタン + トリガー（ID: 0x101）
  msg.id = 0x101;
  msg.len = 4;
  
  ControllerFrame2 frame2;
  frame2.buttons = (uint16_t)buttons;
  frame2.l2 = (uint8_t)l2;
  frame2.r2 = (uint8_t)r2;
  
  memcpy(msg.buf, &frame2, sizeof(frame2));
  Can1.write(msg);
}


void setup() {
  Serial.begin(9600);
  Serial4.begin(115200);
  Can1.begin();
  Can1.setBaudRate(500000);
}

enum systemMode{
  MANUAL = 0,
  AUTO = 1
} mode = AUTO;

void loop() {
  if(SerialRead()) {
    sendControllerDataToCAN();
    if(button.options) mode = MANUAL;
    if(button.share) mode = AUTO;
  }
  switch (mode) {
    case MANUAL:
      Serial.println("Manual Mode");
      // Manual mode logic here
      break;
    case AUTO:
      Serial.println("Auto Mode");
      // Auto mode logic here
      break;
  }
  
  delay(50);
}