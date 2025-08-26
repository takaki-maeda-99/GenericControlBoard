#pragma once
#include <FlexCAN_T4.h>
#include <functional>
#include "DjiMotor.hpp"
#include "PID.h"

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
using MotorControlCallback = std::function<void(uint8_t motorIdx, const MotorCommand &cmd, DjiMotorCan<CAN2> &motors)>;
using CommandValidationCallback = std::function<bool(uint8_t motorIdx, const MotorCommand &cmd)>;

/**
 * CANモーターコントローラークラス
 * 
 * 【主な機能】
 * • CANメッセージの解析とモーターコマンドへの変換
 * • シーケンス番号による重複コマンド検出
 * • タイムアウト管理
 * • カスタマイズ可能なモーター制御コールバック
 * • コマンド妥当性検証コールバック
 * 
 * 【基本的な使い方】
 * ```cpp
 * FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
 * FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
 * DjiMotorCan<CAN2> motors(can2, 72.0f);
 * CanMotorController controller(1, motors);
 * 
 * // モーター制御コールバック設定
 * controller.setControlCallback([](uint8_t idx, const MotorCommand &cmd, auto &motors) {
 *     // カスタムモーター制御ロジック
 * });
 * 
 * // CANハンドラー設定
 * can1.onReceive([&controller](const CAN_message_t &msg) {
 *     controller.handleCanMessage(msg);
 * });
 * 
 * // 制御ループ実行
 * void controlLoop() {
 *     controller.executeCommands();
 * }
 * ```
 */
class CanMotorController {
public:
    // コンストラクタ
    // boardId: ボードID (1-4)
    // motors: DJIモーター制御オブジェクト
    // maxMotors: 制御するモーター数 (デフォルト4)
    CanMotorController(uint8_t boardId, DjiMotorCan<CAN2> &motors, uint8_t maxMotors = 4) 
        : myBoardId_(boardId), motors_(motors), maxMotors_(maxMotors) {
        // コマンド配列初期化
        for (int i = 0; i < 8; i++) {
            motorCommands_[i] = MotorCommand();
        }
    }

    // CANメッセージ処理
    bool handleCanMessage(const CAN_message_t &msg) {
        MotorCommand cmd;
        if (parseMotorCommand(msg, cmd)) {
            uint8_t idx = cmd.motorId - 1;  // 0-7インデックス

            // シーケンス番号チェック（重複コマンド拒否）
            if (cmd.seq == motorCommands_[idx].seq && motorCommands_[idx].lastUpdate > 0) {
                return false; // 同じシーケンス番号は無視
            }

            // 妥当性検証コールバック実行
            if (validationCallback_ && !validationCallback_(idx, cmd)) {
                return false; // コマンドをブロック
            }

            // コマンド更新
            motorCommands_[idx] = cmd;
            
            if (debugOutput_) {
                Serial.printf("Motor%d: mode=%02X, val=%.3f, seq=%d\n", 
                             cmd.motorId, cmd.mode, convertToFloat(cmd.value), cmd.seq);
            }
            return true;
        }
        return false;
    }

    // モーター制御実行（ISRまたはメインループから呼び出し）
    void executeCommands() {
        for (uint8_t i = 0; i < maxMotors_; i++) {
            const MotorCommand &cmd = motorCommands_[i];
            
            if (!isCommandValid(cmd)) {
                // タイムアウト時は停止
                motors_.sendCurrent(i + 1, 0);
                continue;
            }
            
            // コントロールコールバック実行
            if (controlCallback_) {
                controlCallback_(i, cmd, motors_);
            } else {
                // デフォルトの制御（停止）
                motors_.sendCurrent(i + 1, 0);
            }
        }
        
        motors_.flush(); // 全コマンド送信
    }

    // モーター制御コールバック設定
    void setControlCallback(MotorControlCallback callback) {
        controlCallback_ = callback;
    }

    // コマンド妥当性検証コールバック設定
    void setValidationCallback(CommandValidationCallback callback) {
        validationCallback_ = callback;
    }

    // デバッグ出力ON/OFF
    void setDebugOutput(bool enable) {
        debugOutput_ = enable;
    }

    // 最新のモーターコマンド取得
    const MotorCommand& getCommand(uint8_t motorId) const {
        if (motorId >= 1 && motorId <= 8) {
            return motorCommands_[motorId - 1];
        }
        static MotorCommand empty;
        return empty;
    }

    // ユーティリティ関数
    static float convertToFloat(int32_t value) {
        return (float)value / 1000.0f;
    }
    
    static int32_t convertToInt32(float value) {
        return (int32_t)(value * 1000.0f);
    }

private:
    uint8_t myBoardId_;
    DjiMotorCan<CAN2> &motors_;
    uint8_t maxMotors_;
    MotorCommand motorCommands_[8];
    MotorControlCallback controlCallback_;
    CommandValidationCallback validationCallback_;
    bool debugOutput_ = false;

    // CANメッセージ解析
    bool parseMotorCommand(const CAN_message_t &msg, MotorCommand &cmd) {
        // CANメッセージIDから BoardId と MotorId を抽出
        uint8_t boardId = (msg.id >> 4) & 0x0F;  // 上位4bit
        uint8_t motorId = msg.id & 0x0F;         // 下位4bit
        
        // 自分宛てでない、またはモーターID範囲外
        if (boardId != myBoardId_ || motorId < 1 || motorId > 8) {
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

    // コマンドタイムアウトチェック
    bool isCommandValid(const MotorCommand &cmd) {
        if (cmd.lastUpdate == 0) return false; // 未初期化
        
        uint32_t timeoutMs = (cmd.timeout == 0) ? 100 : (cmd.timeout * 10); // デフォルト100ms
        return (millis() - cmd.lastUpdate) < timeoutMs;
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