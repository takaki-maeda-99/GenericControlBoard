#pragma once
#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "DjiMotor.hpp"
#include "PID.h"

struct MotorCommand {
    uint8_t motorId;
    uint8_t mode;
    uint8_t opt;
    int32_t value;
    uint8_t seq;
    uint8_t timeout;
    uint32_t lastUpdate;
    
    MotorCommand() : motorId(0), mode(0x00), opt(0), value(0), seq(0), timeout(0), lastUpdate(0) {}
} __attribute__((packed));

template<CAN_DEV_TABLE CAN_BUS>
class CANMotorControl {
public:
    CANMotorControl(FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16>& canBus, 
                   DjiMotorCan<CAN2>& motors, uint8_t boardId);
    
    void begin();
    void sendCAN(uint8_t boardId, uint8_t motorId, uint8_t mode, float value, uint8_t timeout = 10);
    bool parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd);
    bool isCommandValid(const MotorCommand &cmd);
    float convertToFloat(int32_t value);
    int32_t convertToInt32(float value);
    
    void handleMotorCommand(const CAN_message_t &msg);
    static void canReceiveHandler(const CAN_message_t &msg);
    
    void executeSpeedControl(uint8_t motorIdx, float targetSpeed, Pid& speedPid);
    void executeAngleControl(uint8_t motorIdx, float targetAngle, bool bounded, 
                           Pid& anglePid, Pid& speedPid);
    
    void processMotorCommand(uint8_t motorIdx, Pid* anglePid, Pid* speedPid);
    
    MotorCommand motorCommands[8];
    static CANMotorControl<CAN_BUS>* instance_;
    
private:
    FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16>& canBus_;
    DjiMotorCan<CAN2>& motors_;
    uint8_t myBoardId_;
    static uint8_t seqCounter_;
};

template<CAN_DEV_TABLE CAN_BUS>
uint8_t CANMotorControl<CAN_BUS>::seqCounter_ = 0;

template<CAN_DEV_TABLE CAN_BUS>
CANMotorControl<CAN_BUS>* CANMotorControl<CAN_BUS>::instance_ = nullptr;

template<CAN_DEV_TABLE CAN_BUS>
CANMotorControl<CAN_BUS>::CANMotorControl(FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16>& canBus, 
                               DjiMotorCan<CAN2>& motors, uint8_t boardId) 
    : canBus_(canBus), motors_(motors), myBoardId_(boardId) {
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::begin() {
    canBus_.begin();
    canBus_.setBaudRate(500000);
    canBus_.enableFIFO();
    canBus_.enableFIFOInterrupt();
    instance_ = this;
    canBus_.onReceive(FIFO, canReceiveHandler);
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::sendCAN(uint8_t boardId, uint8_t motorId, uint8_t mode, float value, uint8_t timeout) {
    CAN_message_t msg;
    msg.id = (boardId << 4) | motorId;
    msg.len = 8;
    
    int32_t intValue = convertToInt32(value);
    
    msg.buf[0] = mode;
    msg.buf[1] = 0x00;
    msg.buf[2] = intValue & 0xFF;
    msg.buf[3] = (intValue >> 8) & 0xFF;
    msg.buf[4] = (intValue >> 16) & 0xFF;
    msg.buf[5] = (intValue >> 24) & 0xFF;
    msg.buf[6] = seqCounter_++;
    msg.buf[7] = timeout;
    
    canBus_.write(msg);
}

template<CAN_DEV_TABLE CAN_BUS>
bool CANMotorControl<CAN_BUS>::parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd) {
    uint8_t boardId = (msg.id >> 4) & 0x0F;
    uint8_t motorId = msg.id & 0x0F;
    
    if (boardId != myBoardId_ || motorId < 1 || motorId > 8) {
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

template<CAN_DEV_TABLE CAN_BUS>
bool CANMotorControl<CAN_BUS>::isCommandValid(const MotorCommand &cmd) {
    if (cmd.lastUpdate == 0) return false;
    
    uint32_t timeoutMs = (cmd.timeout == 0) ? 100 : (cmd.timeout * 10);
    return (millis() - cmd.lastUpdate) < timeoutMs;
}

template<CAN_DEV_TABLE CAN_BUS>
float CANMotorControl<CAN_BUS>::convertToFloat(int32_t value) {
    return (float)value / 1000.0f;
}

template<CAN_DEV_TABLE CAN_BUS>
int32_t CANMotorControl<CAN_BUS>::convertToInt32(float value) {
    return (int32_t)(value * 1000.0f);
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::handleMotorCommand(const CAN_message_t &msg) {
    MotorCommand cmd;
    if (parseMotorCommand(msg, cmd)) {
        uint8_t idx = cmd.motorId - 1;
        
        if (cmd.seq == motorCommands[idx].seq && motorCommands[idx].lastUpdate > 0) {
            return;
        }
        
        motorCommands[idx] = cmd;
    }
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::canReceiveHandler(const CAN_message_t &msg) {
    if (instance_) {
        instance_->handleMotorCommand(msg);
    }
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::executeSpeedControl(uint8_t motorIdx, float targetSpeed, Pid& speedPid) {
    const auto &fb = motors_.feedback(motorIdx + 1);
    int16_t speedCmd = speedPid.compute(fb.getSpeedRadiansPerSec(), targetSpeed);
    motors_.sendCurrent(motorIdx + 1, speedCmd);
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::executeAngleControl(uint8_t motorIdx, float targetAngle, bool bounded, 
                                        Pid& anglePid, Pid& speedPid) {
    const auto &fb = motors_.feedback(motorIdx + 1);
    const float fbAngle = fb.getAngleRadiansWrapped();
    const float fbSpeed = fb.getSpeedRadiansPerSec();

    if (bounded) {
        targetAngle = constrain(targetAngle, -PI, PI);
    } else {
        if (targetAngle - fbAngle > PI) {
            targetAngle -= 2*PI;
        } else if (targetAngle - fbAngle < -PI) {
            targetAngle += 2*PI;
        }
    }

    float targetSpeed = anglePid.compute(fbAngle, targetAngle);
    int16_t speedCmd = speedPid.compute(fbSpeed, targetSpeed);
    
    motors_.sendCurrent(motorIdx + 1, speedCmd);
}

template<CAN_DEV_TABLE CAN_BUS>
void CANMotorControl<CAN_BUS>::processMotorCommand(uint8_t motorIdx, Pid* anglePid, Pid* speedPid) {
    const MotorCommand &cmd = motorCommands[motorIdx];
    
    if (!isCommandValid(cmd)) {
        motors_.sendCurrent(motorIdx + 1, 0);
        return;
    }
    
    float value = convertToFloat(cmd.value);
    
    switch (cmd.mode) {
        case 0x00:
            motors_.sendCurrent(motorIdx + 1, 0);
            break;
        case 0x01:
            if (speedPid) executeSpeedControl(motorIdx, value, *speedPid);
            break;
        case 0x02:
            if (anglePid && speedPid) executeAngleControl(motorIdx, value, true, *anglePid, *speedPid);
            break;
        case 0x03:
            if (anglePid && speedPid) executeAngleControl(motorIdx, value, false, *anglePid, *speedPid);
            break;
        default:
            motors_.sendCurrent(motorIdx + 1, 0);
            break;
    }
}