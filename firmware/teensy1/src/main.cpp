#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

volatile int state = 0;
volatile int buttons = 0;
volatile int lx = 0, ly = 0;
volatile int rx = 0, ry = 0;
volatile int l2 = 0, r2 = 0;

struct motorCmd
{
  uint8_t id;
  uint8_t speed;
  int16_t position;
};

struct ControllerData
{
  uint8_t state;
  uint16_t buttons;
  int8_t lx, ly;
  int8_t rx, ry;
  uint8_t l2, r2;
};


void sendControllerDataToCAN() {
  CAN_message_t msg;
  msg.id = 0x100;
  msg.len = 8;
  
  ControllerData data;
  data.state = (uint8_t)state;
  data.buttons = (uint16_t)buttons;
  data.lx = (int8_t)lx;
  data.ly = (int8_t)ly;
  data.rx = (int8_t)rx;
  data.ry = (int8_t)ry;
  data.l2 = (uint8_t)l2;
  data.r2 = (uint8_t)r2;
  
  memcpy(msg.buf, &data, sizeof(data));
  Can1.write(msg);
  
  Serial.println("Controller data sent via CAN");
}

void setup() {
  Serial.begin(57600);
  Serial4.begin(115200);
  Can1.begin();
  Can1.setBaudRate(500000);
  Serial.println("CAN Bus initialized");
}

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

    return true;
  }
  return false;

  // values[0]: state (0=READY, 1=OVERCURRENT, 2=UNDERVOLTAGE, 3=NOT_CONNECTED, 4=E_STOP)
  // values[1]: buttons ビット配置
  // values[2]: lx (-128~127), values[3]: ly (-128~127)
  // values[4]: rx (-128~127), values[5]: ry (-128~127)
  // values[6]: l2 (0~255), values[7]: r2 (0~255)
  
  // if(values[1] & (1<<0)) Serial.println("Square pressed");
  // if(values[1] & (1<<1)) Serial.println("Cross pressed");
  // if(values[1] & (1<<2)) Serial.println("Circle pressed");
  // if(values[1] & (1<<3)) Serial.println("Triangle pressed");
  // if(values[1] & (1<<4)) Serial.println("L1 pressed");
  // if(values[1] & (1<<5)) Serial.println("R1 pressed");
  // if(values[1] & (1<<6)) Serial.println("L3 pressed");
  // if(values[1] & (1<<7)) Serial.println("R3 pressed");
  // if(values[1] & (1<<8)) Serial.println("Share pressed");
  // if(values[1] & (1<<9)) Serial.println("Options pressed");
  // if(values[1] & (1<<10)) Serial.println("PS pressed");
  // if(values[1] & (1<<11)) Serial.println("Touchpad pressed");
  // if(values[1] & (1<<12)) Serial.println("Up pressed");
  // if(values[1] & (1<<13)) Serial.println("Down pressed");
  // if(values[1] & (1<<14)) Serial.println("Left pressed");
  // if(values[1] & (1<<15)) Serial.println("Right pressed");
}

/*
// CAN受信処理の例
void readCANMessage() {
  CAN_message_t msg;
  if(Can1.read(msg)) {
    if(msg.id == 0x100) {  // コントローラデータ
      ControllerData data;
      memcpy(&data, msg.buf, sizeof(data));
      
      // グローバル変数に格納
      state = data.state;
      buttons = data.buttons;
      lx = data.lx;
      ly = data.ly;
      rx = data.rx;
      ry = data.ry;
      l2 = data.l2;
      r2 = data.r2;
      
      Serial.println("Controller data received via CAN");
    }
    else if(msg.id == 0x200) {  // モーターコマンド例
      motorCmd cmd;
      memcpy(&cmd, msg.buf, sizeof(cmd));
      
      Serial.print("Motor ID: ");
      Serial.print(cmd.id);
      Serial.print(", Speed: ");
      Serial.print(cmd.speed);
      Serial.print(", Position: ");
      Serial.println(cmd.position);
    }
  }
}
*/

void loop() {
  if(SerialRead()) {
    sendControllerDataToCAN();
  }
  
  // CAN受信処理を有効にする場合は以下をコメントアウト
  // readCANMessage();
  
  delay(50);
}