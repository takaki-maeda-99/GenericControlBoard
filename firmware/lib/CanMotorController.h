#pragma once
#include <FlexCAN_T4.h>
#include <functional>

// CANモーターコマンド構造体
struct MotorCommand {
    uint8_t motorId;     // モーターID (1-8)
    uint8_t mode;        // 0x00=Disable, 0x01=Speed, 0x02=BoundedAngle, 0x03=UnboundedAngle, 0xFF=Homing
    uint8_t opt;         // 将来拡張/フラグ (現状0固定)
    int32_t value;       // 指令値 (rad/s または rad)
    uint8_t seq;         // シーケンス番号 (0-255巡回)
    uint8_t timeout;     // タイムアウト [10ms単位] (0=ボード既定)
    uint32_t lastUpdate; // 最終更新時刻 [ms]
    
    MotorCommand() : motorId(0), mode(0x00), opt(0), value(0), seq(0), timeout(0), lastUpdate(0) {}
} __attribute__((packed));

// コールバック関数型定義
using MotorControlCallback = std::function<void(uint8_t motorIdx, const MotorCommand &cmd)>;

class CanMotorController {
public:
    CanMotorController(uint8_t boardId, uint8_t maxMotors = 4) 
        : myBoardId_(boardId), maxMotors_(maxMotors) {
        motorCommands_ = new MotorCommand[maxMotors_];
        for (int i = 0; i < maxMotors_; i++) {
            motorCommands_[i] = MotorCommand();
        }
    }

    ~CanMotorController() {
        delete[] motorCommands_;
    }

    bool handleCanMessage(const CAN_message_t &msg) {
        MotorCommand cmd;
        if (parseMotorCommand(msg, cmd)) {
            uint8_t idx = cmd.motorId - 1;

            if (cmd.seq == motorCommands_[idx].seq && motorCommands_[idx].lastUpdate > 0) {
                return false;
            }

            motorCommands_[idx] = cmd;
            
            if (controlCallback_) {
                controlCallback_(idx, cmd);
            }
            
            return true;
        }
        return false;
    }

    void setControlCallback(MotorControlCallback callback) {
        controlCallback_ = callback;
    }

    static float convertToFloat(int32_t value) {
        return (float)value / 1000.0f;
    }
    
    static int32_t convertToInt32(float value) {
        return (int32_t)(value * 1000.0f);
    }

private:
    uint8_t myBoardId_;
    uint8_t maxMotors_;
    MotorCommand* motorCommands_;
    MotorControlCallback controlCallback_;

    bool parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd) {
        uint8_t boardId = (msg.id >> 4) & 0x0F;
        uint8_t motorId = msg.id & 0x0F;
        
        if (boardId != myBoardId_ || motorId < 1 || motorId > maxMotors_) {
            return false;
        }
        
        if (msg.len < 8) {
            return false;
        }
        
        cmd.motorId = motorId;
        cmd.mode = msg.buf[0];
        cmd.opt = msg.buf[1];
        
        cmd.value = (int32_t)msg.buf[2] | 
                    ((int32_t)msg.buf[3] << 8) | 
                    ((int32_t)msg.buf[4] << 16) | 
                    ((int32_t)msg.buf[5] << 24);
        
        cmd.seq = msg.buf[6];
        cmd.timeout = msg.buf[7];
        cmd.lastUpdate = millis();
        
        return true;
    }
};

// CAN送信ヘルパー関数
inline void sendCanCommand(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can, 
                          uint8_t boardId, uint8_t motorId, uint8_t mode, 
                          float value, uint8_t timeout = 10) {
    CAN_message_t msg;
    msg.id = (boardId << 4) | motorId;
    msg.len = 8;
    
    static uint8_t seq = 0;
    int32_t intValue = CanMotorController::convertToInt32(value);
    
    msg.buf[0] = mode;
    msg.buf[1] = 0x00;    // OPT
    msg.buf[2] = intValue & 0xFF;
    msg.buf[3] = (intValue >> 8) & 0xFF;
    msg.buf[4] = (intValue >> 16) & 0xFF;
    msg.buf[5] = (intValue >> 24) & 0xFF;
    msg.buf[6] = seq++;
    msg.buf[7] = timeout;
    
    can.write(msg);
}