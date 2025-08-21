#pragma once
#include <FlexCAN_T4.h>
#include <cstdint>
#include <cstring>

// パフォーマンス最適化マクロ
#ifdef __GNUC__
    #define DJI_FORCE_INLINE __attribute__((always_inline)) inline
    #define DJI_HOT __attribute__((hot))
    #define DJI_LIKELY(x) __builtin_expect(!!(x), 1)
    #define DJI_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
    #define DJI_FORCE_INLINE inline
    #define DJI_HOT
    #define DJI_LIKELY(x) (x)
    #define DJI_UNLIKELY(x) (x)
#endif

// コンパイル時定数
namespace DjiConstants {
    constexpr uint16_t ENCODER_CPR = 8192;          // エンコーダ分解能
    constexpr uint16_t HALF_ENCODER_CPR = 4096;     // エンコーダ分解能の半分
    constexpr uint32_t TIMEOUT_CHECK_INTERVAL = 10000UL; // タイムアウトチェック間隔 [us]
    constexpr float DJI_DEG_TO_RAD = 0.017453292519943295f; // 度→ラジアン変換
    constexpr float DJI_RAD_TO_DEG = 57.29577951308232f;    // ラジアン→度変換
    constexpr float PI_2 = 6.283185307179586f;      // 2π
}

/**
 * DJIモーターフィードバック構造体
 * モーター1台分の最新状態を保持
 */
struct DjiFeedback {
    uint16_t angleRaw;       // 0-8191: エンコーダ生値
    int16_t  speedRaw;       // [rpm] 回転速度
    int16_t  current;        // [mA] トルク電流
    int32_t  positionCnt;    // 多回転積算角度
    uint32_t lastUpdateTime; // 最終更新時刻 [us]
    uint16_t lastAngleRaw;   // 前回エンコーダ値 (内部用)
    bool     isTimeout;      // タイムアウト状態
    
    DjiFeedback() : angleRaw(0), speedRaw(0), current(0), positionCnt(0), 
                    lastUpdateTime(0), lastAngleRaw(0), isTimeout(true) {}
};

/**
 * DJIモーターCANドライバクラス
 * 
 * 【主な機能】
 * • 最大8台のモーターを制御 (ID 1-8)
 * • タイムアウト検出による安全停止
 * • 多回転位置の自動積算
 * • 設定可能ギア比
 * • 高性能最適化済み
 * 
 * 【CANフレーム】
 * • 送信: 0x200(ID1-4), 0x1FF(ID5-8)
 * • 受信: 0x201-0x208 (各モーターID)
 * 
 * 【基本的な使い方】
 * ```cpp
 * FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
 * DjiMotorCan<CAN2> motors(can2, 72.0f); // 72:1ギア比
 * 
 * void loop() {
 *   // 電流指令
 *   motors.sendCurrent(1, 5000);  // 5A指令
 *   motors.flush();               // 送信
 *   
 *   // フィードバック取得 (自動更新済み)
 *   const auto &fb = motors.feedback(1);
 *   float angle = motors.countToDegrees(fb.positionCnt);
 *   
 *   delay(10); // 制御周期
 * }
 * ```
 */
template<CAN_DEV_TABLE CAN_BUS>
class DjiMotorCan {
public:
    // コンストラクタ - CANバス初期化
    // gearRatio: モーターのギア比 (デフォルト 72:1)
    explicit DjiMotorCan(FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16> &bus, float gearRatio = 72.0f) 
        : bus_(bus), gearRatio_(gearRatio) {
        // 送信フレームバッファ初期化
        for (auto &f : txFrame_) {
            f = CAN_message_t{};
        }
        // コールバック用の静的インスタンスポインタ設定
        _self = this;
        // CAN初期化とコールバック登録
        bus_.begin();
        bus_.setBaudRate(1'000'000);      // DJI推奨 1 Mbps
        bus_.enableFIFO();
        bus_.enableFIFOInterrupt();
        bus_.onReceive(isr);              // 受信割り込み時にisr関数を呼ぶ
    }

    // モーター電流指令設定
    // motorId: 1-8, currentCmd: ±16384mA
    // タイムアウト時は自動的に0mAに設定
    bool sendCurrent(uint8_t motorId, int16_t currentCmd) {
        if (motorId < 1 || motorId > 8) return false;  // ID範囲外は失敗
        
        // タイムアウトチェック
        updateTimeoutStatus();
        const uint8_t idx = motorId - 1;
        if (fb_[idx].isTimeout) {
            currentCmd = 0;  // タイムアウト時は電流0で安全停止
        }
        
        const uint8_t group = (motorId - 1) / 4;  // グループ（0: ID1-4→0x200, 1: ID5-8→0x1FF）
        const uint8_t slot  = (motorId - 1) % 4;  // グループ内のスロット(0～3番目)
        CAN_message_t &frm = txFrame_[group];
        frm.id  = 0x200 - group;                 // group=0 -> 0x200, group=1 -> 0x1FF
        frm.len = 8;
        // 2バイトの電流指令値をビッグエンディアンでフレームに格納
        frm.buf[slot * 2]     = static_cast<uint8_t>(currentCmd >> 8);
        frm.buf[slot * 2 + 1] = static_cast<uint8_t>(currentCmd & 0xFF);
        
        return !fb_[idx].isTimeout;  // タイムアウト時はfalseを返す
    }

    // 電流指令フレーム送信
    // sendCurrent()で設定した値をCANバスに送信
    bool flush() {
        // ループ展開（2フレーム固定のため）
        bool ok = true;
        if (__builtin_expect(txFrame_[0].len != 0, 1)) {
            ok &= bus_.write(txFrame_[0]);
        }
        if (__builtin_expect(txFrame_[1].len != 0, 1)) {
            ok &= bus_.write(txFrame_[1]);
        }
        return ok;
    }


    // フィードバックデータ取得
    // motorId: 1-8
    const DjiFeedback& feedback(uint8_t motorId) const {
        return fb_[motorId - 1];
    }

    // タイムアウト状態更新 (内部用)
    void updateTimeoutStatus(uint32_t timeoutMs = 100) {
        static uint32_t lastCheck = 0;
        const uint32_t now = micros();
        
        // 10ms間隔でのみチェック（パフォーマンス最適化）
        if (now - lastCheck < DjiConstants::TIMEOUT_CHECK_INTERVAL) return;
        lastCheck = now;
        
        const uint32_t timeoutUs = timeoutMs * 1000UL;
        
        // ループアンローリングで高速化（8モーター固定なので）
        fb_[0].isTimeout = (fb_[0].lastUpdateTime == 0) || (now - fb_[0].lastUpdateTime) > timeoutUs;
        fb_[1].isTimeout = (fb_[1].lastUpdateTime == 0) || (now - fb_[1].lastUpdateTime) > timeoutUs;
        fb_[2].isTimeout = (fb_[2].lastUpdateTime == 0) || (now - fb_[2].lastUpdateTime) > timeoutUs;
        fb_[3].isTimeout = (fb_[3].lastUpdateTime == 0) || (now - fb_[3].lastUpdateTime) > timeoutUs;
        fb_[4].isTimeout = (fb_[4].lastUpdateTime == 0) || (now - fb_[4].lastUpdateTime) > timeoutUs;
        fb_[5].isTimeout = (fb_[5].lastUpdateTime == 0) || (now - fb_[5].lastUpdateTime) > timeoutUs;
        fb_[6].isTimeout = (fb_[6].lastUpdateTime == 0) || (now - fb_[6].lastUpdateTime) > timeoutUs;
        fb_[7].isTimeout = (fb_[7].lastUpdateTime == 0) || (now - fb_[7].lastUpdateTime) > timeoutUs;
    }

    // 角度リセット (原点設定)
    // motorId: 1-8, resetAngleDeg: 設定角度[deg]
    void resetAngle(uint8_t motorId, float resetAngleDeg) {
        if (motorId < 1 || motorId > 8) return;
        const uint8_t idx = motorId - 1;
        // 角度[deg]をエンコーダのカウント値に換算 (1回転=8192カウント, 設定されたギア比使用)
        const float cnt2deg = 360.0f / (8192.0f * gearRatio_);
        const int32_t resetCnt = static_cast<int32_t>(resetAngleDeg / cnt2deg);
        // 現在のエンコーダ生値を基準に積算位置カウントを調整
        fb_[idx].positionCnt = resetCnt;
        fb_[idx].lastAngleRaw = fb_[idx].angleRaw;
    }

    // ギア比設定
    void setGearRatio(float gearRatio) {
        if (gearRatio > 0.0f) {
            gearRatio_ = gearRatio;
        }
    }

    // ギア比取得
    float getGearRatio() const {
        return gearRatio_;
    }

    // カウント → 角度[deg] 変換
    DJI_FORCE_INLINE DJI_HOT float countToDegrees(int32_t positionCnt) const {
        // 事前計算した変換係数を使用
        const float conversion = 360.0f / (DjiConstants::ENCODER_CPR * gearRatio_);
        return positionCnt * conversion;
    }

    // 角度[deg] → カウント 変換
    DJI_FORCE_INLINE DJI_HOT int32_t degreesToCount(float degrees) const {
        // 事前計算した変換係数を使用
        const float conversion = (DjiConstants::ENCODER_CPR * gearRatio_) / 360.0f;
        return static_cast<int32_t>(degrees * conversion);
    }

    // カウント → 角度[rad] 変換
    DJI_FORCE_INLINE DJI_HOT float countToRadians(int32_t positionCnt) const {
        const float conversion = DjiConstants::PI_2 / (DjiConstants::ENCODER_CPR * gearRatio_);
        return positionCnt * conversion;
    }

private:
    // CAN受信割り込みハンドラ
    static void isr(const CAN_message_t &msg) {
        if (_self) _self->processMessage(msg);
    }

    // フィードバック処理 (割り込み内で自動実行)
    void processMessage(const CAN_message_t &msg) {
        if (DJI_UNLIKELY(msg.id < 0x201 || msg.id > 0x208)) return;
        
        const uint8_t idx = msg.id - 0x201;
        DjiFeedback &fb_ref = fb_[idx];
        
        // データ展開
        const uint16_t ang  = (static_cast<uint16_t>(msg.buf[0]) << 8) | msg.buf[1];
        const int16_t  rpm  = (static_cast<int16_t>(msg.buf[2]) << 8) | msg.buf[3];
        const int16_t  curr = (static_cast<int16_t>(msg.buf[4]) << 8) | msg.buf[5];
        
        // 多回転処理
        const int16_t delta_raw = ang - fb_ref.lastAngleRaw;
        int16_t delta;
        
        if (DJI_LIKELY(delta_raw >= -DjiConstants::HALF_ENCODER_CPR && delta_raw <= DjiConstants::HALF_ENCODER_CPR)) {
            delta = delta_raw;
        } else {
            delta = (delta_raw > DjiConstants::HALF_ENCODER_CPR) ? 
                   (delta_raw - DjiConstants::ENCODER_CPR) : 
                   (delta_raw + DjiConstants::ENCODER_CPR);
        }
        
        // データ更新
        fb_ref.positionCnt    += delta;
        fb_ref.lastAngleRaw    = ang;
        fb_ref.angleRaw        = ang;
        fb_ref.speedRaw        = rpm;
        fb_ref.current         = curr;
        fb_ref.lastUpdateTime  = micros();
        fb_ref.isTimeout       = false;
    }

    FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16> &bus_;  // CANバス参照
    CAN_message_t txFrame_[2];     // 送信バッファ [0x200, 0x1FF]
    DjiFeedback   fb_[8];          // モーター1-8のフィードバック
    float         gearRatio_;      // ギア比
    static DjiMotorCan<CAN_BUS>* _self;  // 割り込み用インスタンス
};

// 静的メンバの実体をテンプレートごとに定義
template<CAN_DEV_TABLE CAN_BUS>
DjiMotorCan<CAN_BUS>* DjiMotorCan<CAN_BUS>::_self = nullptr;
