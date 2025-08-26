#pragma once
#include <Arduino.h>
#include <functional>
#include "DjiMotor.hpp"

// ホーミング状態定義
enum HomingState {
    HOMING_IDLE = 0,           // ホーミング未実行
    HOMING_IN_PROGRESS = 1,    // ホーミング実行中
    HOMING_COMPLETED = 2       // ホーミング完了
};

// ホーミングデータ構造体
struct HomingData {
    HomingState state;
    uint32_t startTime;
    float homeAngle;
    bool completed;
    
    HomingData() : state(HOMING_IDLE), startTime(0), homeAngle(0), completed(false) {}
};

// ホーミング手順コールバック型定義
using HomingProcedureCallback = std::function<bool(uint8_t motorIdx, DjiMotorCan<CAN2> &motors)>;
using HomingCompletionCallback = std::function<void(uint8_t motorIdx, float homeAngle, DjiMotorCan<CAN2> &motors)>;

/**
 * 共通ホーミングコントローラークラス
 * 
 * 【主な機能】
 * • ホーミング状態管理（IDLE, IN_PROGRESS, COMPLETED）
 * • ホーミングコマンド処理（0xFFコマンド対応）
 * • カスタマイズ可能なホーミング手順コールバック
 * • ホーミング完了処理
 * 
 * 【使い方】
 * ```cpp
 * HomingController homing(4, homeAngles);
 * 
 * // ボード固有のホーミング手順を設定
 * homing.setProcedureCallback([](uint8_t idx, auto &motors) {
 *     // 各ボード固有のホーミング処理
 *     return isHomingComplete; // true=完了, false=継続
 * });
 * 
 * // ホーミング制御実行
 * homing.update(motors);
 * ```
 */
class HomingController {
public:
    // コンストラクタ
    // maxMotors: 制御するモーター数
    // homeAngles: 各モーターのホーム角度配列
    HomingController(uint8_t maxMotors, const float* homeAngles) 
        : maxMotors_(maxMotors) {
        // ホーミングデータ配列初期化
        for (uint8_t i = 0; i < 8; i++) {
            homingData_[i] = HomingData();
            if (i < maxMotors && homeAngles) {
                homingData_[i].homeAngle = homeAngles[i];
            }
        }
    }

    // ホーミング開始
    void startHoming(uint8_t motorIdx) {
        if (motorIdx >= maxMotors_) return;
        
        homingData_[motorIdx].state = HOMING_IN_PROGRESS;
        homingData_[motorIdx].startTime = millis();
        homingData_[motorIdx].completed = false;
        
        Serial.printf("Starting homing for motor %d\n", motorIdx + 1);
    }

    // 全モーターのホーミング開始
    void startAllHoming() {
        for (uint8_t i = 0; i < maxMotors_; i++) {
            startHoming(i);
        }
    }

    // ホーミング状態取得
    HomingState getState(uint8_t motorIdx) const {
        if (motorIdx >= maxMotors_) return HOMING_IDLE;
        return homingData_[motorIdx].state;
    }

    // ホーミング完了チェック
    bool isHomingComplete(uint8_t motorIdx) const {
        if (motorIdx >= maxMotors_) return false;
        return homingData_[motorIdx].state == HOMING_COMPLETED;
    }

    // 全モーターホーミング完了チェック
    bool isAllHomingComplete() const {
        for (uint8_t i = 0; i < maxMotors_; i++) {
            if (!isHomingComplete(i)) return false;
        }
        return true;
    }

    // ホーミング制御更新（メインループまたはISRから呼び出し）
    void update(DjiMotorCan<CAN2> &motors) {
        bool anyHoming = false;
        
        for (uint8_t i = 0; i < maxMotors_; i++) {
            if (homingData_[i].state == HOMING_IN_PROGRESS) {
                // ボード固有のホーミング手順実行
                bool completed = false;
                if (procedureCallback_) {
                    completed = procedureCallback_(i, motors);
                } else {
                    // デフォルト処理（停止）
                    motors.sendCurrent(i + 1, 0);
                    completed = true;
                }
                
                if (completed) {
                    // ホーミング完了処理
                    homingData_[i].state = HOMING_COMPLETED;
                    homingData_[i].completed = true;
                    
                    // 角度リセット
                    motors.resetAngle(i + 1, homingData_[i].homeAngle);
                    motors.sendCurrent(i + 1, 0);
                    
                    // 完了コールバック実行
                    if (completionCallback_) {
                        completionCallback_(i, homingData_[i].homeAngle, motors);
                    }
                    
                    Serial.printf("Motor %d homing completed\n", i + 1);
                }
                
                anyHoming = true;
            }
        }
        
        if (anyHoming) {
            motors.flush(); // ホーミング中のモーターがある場合のみ送信
        }
    }

    // 0xFFコマンド処理（CanMotorControllerコールバックから呼び出し）
    void handleHomingCommand(uint8_t motorIdx) {
        if (motorIdx >= maxMotors_) return;
        
        if (homingData_[motorIdx].state != HOMING_IN_PROGRESS) {
            startHoming(motorIdx);
        }
    }

    // モーター制御許可チェック（ホーミング完了必須）
    bool isControlAllowed(uint8_t motorIdx) const {
        return isHomingComplete(motorIdx);
    }

    // ホーミング手順コールバック設定
    void setProcedureCallback(HomingProcedureCallback callback) {
        procedureCallback_ = callback;
    }

    // ホーミング完了コールバック設定
    void setCompletionCallback(HomingCompletionCallback callback) {
        completionCallback_ = callback;
    }

    // ホーミング初期化
    void initialize() {
        for (uint8_t i = 0; i < maxMotors_; i++) {
            homingData_[i].state = HOMING_IDLE;
            homingData_[i].completed = false;
            homingData_[i].startTime = 0;
        }
    }

    // デバッグ情報出力
    void printStatus() const {
        Serial.println("=== Homing Status ===");
        for (uint8_t i = 0; i < maxMotors_; i++) {
            const char* stateStr;
            switch (homingData_[i].state) {
                case HOMING_IDLE: stateStr = "IDLE"; break;
                case HOMING_IN_PROGRESS: stateStr = "IN_PROGRESS"; break;
                case HOMING_COMPLETED: stateStr = "COMPLETED"; break;
                default: stateStr = "UNKNOWN"; break;
            }
            Serial.printf("Motor%d: %s (angle=%.3f)\n", i + 1, stateStr, homingData_[i].homeAngle);
        }
    }

private:
    uint8_t maxMotors_;
    HomingData homingData_[8];
    HomingProcedureCallback procedureCallback_;
    HomingCompletionCallback completionCallback_;
};