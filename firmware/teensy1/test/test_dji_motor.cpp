#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "../src/DjiMotor.hpp"
#include "../src/PID.h"

// CAN2バス（DJIモーター用）
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan<CAN2> motors(can2, 72.0f);  // 72:1 ギア比で初期化

// PID制御器（位置制御用）
SimplePID posPID(25.0, 0.1, 0.05, -16384, 16384);

// テスト用変数
uint32_t lastTestTime = 0;
uint8_t testPhase = 0;
int16_t testCurrent = 0;
float targetPosition = 0.0;

void printWelcomeMessage() {
  Serial.println("=====================================");
  Serial.println("    DJI Motor Library Test v1.0");
  Serial.println("=====================================");
  Serial.println("Testing motor ID 1-4 on CAN2 bus");
  Serial.println();
}

void printCommandHelp() {
  Serial.println("Available Commands:");
  Serial.println("  't' - Basic current test (sine wave)");
  Serial.println("  'p' - Position control test (PID)");
  Serial.println("  'm' - Multi-motor test (4 motors)");
  Serial.println("  'o' - Timeout test (safety check)");
  Serial.println("  's' - Stop all motors (emergency)");
  Serial.println("  'r' - Reset motor angles to 0°");
  Serial.println("  'i' - Display motor info");
  Serial.println("  'g' - Set gear ratio");
  Serial.println("  'h' - Show this help");
  Serial.println("  Position test extras:");
  Serial.println("    '+' - Target +90° (in position mode)");
  Serial.println("    '-' - Target -90° (in position mode)");
  Serial.println("-------------------------------------");
  Serial.print("Current status: ");
  switch(testPhase) {
    case 0: Serial.println("IDLE - Ready for commands"); break;
    case 1: Serial.println("RUNNING - Current test"); break;
    case 2: Serial.println("RUNNING - Position control"); break;
    case 3: Serial.println("RUNNING - Multi-motor test"); break;
    case 4: Serial.println("RUNNING - Timeout test"); break;
  }
  Serial.println("=====================================");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  printWelcomeMessage();
  printCommandHelp();
  
  lastTestTime = millis();
}

void basicCurrentTest() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();
  
  // サイン波電流指令テスト
  float t = millis() * 0.001f;
  testCurrent = (int16_t)(2000.0f * sin(t * 0.5f));
  
  bool success = motors.sendCurrent(1, testCurrent);
  motors.flush();
  
  Serial.print("Current Test - Motor 1: ");
  Serial.print(testCurrent);
  Serial.print("mA, Success: ");
  Serial.println(success ? "OK" : "TIMEOUT");
}

void positionControlTest() {
  static uint32_t lastUpdate = 0;
  static bool initialized = false;
  
  if (!initialized) {
    targetPosition = 0.0;  // 0度目標
    initialized = true;
    Serial.println("Position Control Test Started - Target: 0 deg");
  }
  
  if (millis() - lastUpdate < 10) return;  // 100Hz制御
  lastUpdate = millis();
  
  // フィードバック取得
  const auto &fb = motors.feedback(1);
  float currentPos = motors.countToDegrees(fb.positionCnt);  // deg
  
  // PID制御
  int16_t currentCmd = (int16_t)posPID.compute(currentPos, targetPosition);
  
  bool success = motors.sendCurrent(1, currentCmd);
  motors.flush();
  
  // 1秒ごとに表示
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Pos Control - Target: ");
    Serial.print(targetPosition, 1);
    Serial.print("deg, Current: ");
    Serial.print(currentPos, 1);
    Serial.print("deg, Cmd: ");
    Serial.print(currentCmd);
    Serial.print("mA, Success: ");
    Serial.println(success ? "OK" : "TIMEOUT");
  }
}

void multiMotorTest() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 200) return;
  lastUpdate = millis();
  
  float t = millis() * 0.001f;
  int16_t current1 = (int16_t)(1500.0f * sin(t * 0.3f));
  int16_t current2 = (int16_t)(1500.0f * sin(t * 0.3f));
  int16_t current3 = (int16_t)(1500.0f * sin(t * 0.3f));
  int16_t current4 = (int16_t)(1500.0f * sin(t * 0.3f));
  
  bool s1 = motors.sendCurrent(1, current1);
  bool s2 = motors.sendCurrent(2, current2);
  bool s3 = motors.sendCurrent(3, current3);
  bool s4 = motors.sendCurrent(4, current4);
  motors.flush();
  
  Serial.print("Multi-Motor Test - M1:");
  Serial.print(current1);
  Serial.print(s1 ? "(OK) " : "(TO) ");
  Serial.print("M2:");
  Serial.print(current2);
  Serial.print(s2 ? "(OK) " : "(TO) ");
  Serial.print("M3:");
  Serial.print(current3);
  Serial.print(s3 ? "(OK) " : "(TO) ");
  Serial.print("M4:");
  Serial.print(current4);
  Serial.println(s4 ? "(OK)" : "(TO)");
}

void timeoutTest() {
  // モーター1に高電流指令を送り続け、フィードバック停止をシミュレート
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();
  
  bool success = motors.sendCurrent(1, 8000);  // 高電流
  motors.flush();
  
  const auto &fb = motors.feedback(1);
  Serial.print("Timeout Test - Current: 8000mA, Success: ");
  Serial.print(success ? "OK" : "TIMEOUT");
  Serial.print(", Last Update: ");
  Serial.print((millis() - fb.lastUpdateTime/1000));
  Serial.println("ms ago");
}

void stopAllMotors() {
  for (uint8_t id = 1; id <= 8; id++) {
    motors.sendCurrent(id, 0);
  }
  motors.flush();
  Serial.println("All motors stopped");
}

void resetMotorAngles() {
  for (uint8_t id = 1; id <= 4; id++) {
    motors.resetAngle(id, 0.0f);
  }
  Serial.println("Motor angles reset to 0 degrees");
}

void displayMotorInfo() {
  Serial.println("=== Motor Status ===");
  Serial.print("Gear Ratio: ");
  Serial.print(motors.getGearRatio(), 1);
  Serial.println(":1");
  Serial.println("---------------------");
  for (uint8_t id = 1; id <= 4; id++) {
    const auto &fb = motors.feedback(id);
    float angleDeg = motors.countToDegrees(fb.positionCnt);
    
    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(": Pos=");
    Serial.print(angleDeg, 2);
    Serial.print("deg, RPM=");
    Serial.print(fb.speedRaw);
    Serial.print(", Current=");
    Serial.print(fb.current);
    Serial.print("mA, Timeout=");
    Serial.println(fb.isTimeout ? "YES" : "NO");
  }
  Serial.println("==================");
}

void processSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.println(); // 改行
    
    switch (cmd) {
      case 't':
        testPhase = 1;
        Serial.println(">>> Starting basic current test (sine wave)");
        Serial.println("    Motor 1 will receive sine wave current commands");
        Serial.println("    Press 's' to stop");
        break;
      case 'p':
        testPhase = 2;
        Serial.println(">>> Starting position control test (PID)");
        Serial.println("    Motor 1 controlled to 0° position");
        Serial.println("    Use '+'/'-' to change target by ±90°");
        Serial.println("    Press 's' to stop");
        break;
      case 'm':
        testPhase = 3;
        Serial.println(">>> Starting multi-motor test");
        Serial.println("    Motors 1-4 will receive different wave patterns");
        Serial.println("    Press 's' to stop");
        break;
      case 'o':
        testPhase = 4;
        Serial.println(">>> Starting timeout test");
        Serial.println("    Testing safety timeout feature");
        Serial.println("    Press 's' to stop");
        break;
      case 's':
        testPhase = 0;
        stopAllMotors();
        Serial.println(">>> All tests stopped");
        Serial.println("    Ready for new commands");
        break;
      case 'r':
        resetMotorAngles();
        Serial.println(">>> Motor angles reset to 0°");
        break;
      case 'i':
        displayMotorInfo();
        break;
      case 'g':
        Serial.println(">>> Enter gear ratio (e.g., 72 for 72:1):");
        Serial.println("    Common ratios: 1, 19, 36, 72");
        while(!Serial.available()) delay(10);
        {
          float newRatio = Serial.parseFloat();
          if (newRatio > 0 && newRatio < 1000) {
            motors.setGearRatio(newRatio);
            Serial.print(">>> Gear ratio set to: ");
            Serial.print(newRatio, 1);
            Serial.println(":1");
          } else {
            Serial.println(">>> Invalid gear ratio (must be 0.1-999)");
          }
          // Clear remaining input
          while(Serial.available()) Serial.read();
        }
        break;
      case 'h':
        printCommandHelp();
        break;
      case '+':
        if (testPhase == 2) {
          targetPosition += 90.0f;
          Serial.print(">>> Target position changed to: ");
          Serial.print(targetPosition);
          Serial.println("°");
        } else {
          Serial.println(">>> '+' command only works in position control mode (press 'p')");
        }
        break;
      case '-':
        if (testPhase == 2) {
          targetPosition -= 90.0f;
          Serial.print(">>> Target position changed to: ");
          Serial.print(targetPosition);
          Serial.println("°");
        } else {
          Serial.println(">>> '-' command only works in position control mode (press 'p')");
        }
        break;
      case '\n':
      case '\r':
        // 改行コードは無視
        break;
      default:
        Serial.print(">>> Unknown command: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println("    Press 'h' for help");
        break;
    }
  }
}

void processFeedback() {
  // フィードバックは割り込みで自動更新されるため、
  // 特別な処理は不要 (feedback()でいつでもアクセス可能)
}

void loop() {
  processSerialCommands();
  processFeedback();
  
  // 選択されたテストを実行
  switch (testPhase) {
    case 1:
      basicCurrentTest();
      break;
    case 2:
      positionControlTest();
      break;
    case 3:
      multiMotorTest();
      break;
    case 4:
      timeoutTest();
      break;
    default:
      // 待機中は何もしない
      break;
  }
  
  // 定期的にヘルプ表示とモーター情報
  static uint32_t lastInfo = 0;
  static uint32_t lastHelp = 0;
  
  if (testPhase == 0 && millis() - lastHelp > 30000) {
    lastHelp = millis();
    Serial.println("\n[Reminder] Press 'h' for help, or try:");
    Serial.println("  't'=current test, 'p'=position test, 'm'=multi-motor test");
  }
  
  if (testPhase != 0 && millis() - lastInfo > 10000) {
    lastInfo = millis();
    Serial.println("\n=== Periodic Status Update ===");
    displayMotorInfo();
    Serial.println("Press 's' to stop current test\n");
  }
}