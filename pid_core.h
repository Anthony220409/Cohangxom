// ================================================================
//  pid_core.h  —  Pure C/C++ Logic, ZERO Hardware Coupling
//  ESP32-C3 Micro Brushed Drone  |  AUW 45g  |  720+55mm  |  1S
//
//  Unified Header: Constants + CascadedPID Class + Tuning Command Parser
//  Include từ bất kỳ .ino / .cpp nào — all definitions in one file
// ================================================================
#pragma once
#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cmath>

// ================================================================
//  CONDITIONAL COMPILATION: Hardware vs Host (Debug) Mode
// ================================================================
#ifdef PID_DEBUG_HOST
    // Khi debug trên PC: dùng stub Arduino
    #define F(x) x

    class String {
        std::string s;

    public:
        String() = default;
        String(const char* c) : s(c ? c : "") {}
        String(const std::string& x) : s(x) {}

        unsigned int length() const { return static_cast<unsigned int>(s.size()); }

        void trim() {
            auto start = s.find_first_not_of(" \t\r\n");
            if (start == std::string::npos) {
                s.clear();
                return;
            }
            auto end = s.find_last_not_of(" \t\r\n");
            s = s.substr(start, end - start + 1);
        }

        int indexOf(char ch) const {
            auto pos = s.find(ch);
            return pos == std::string::npos ? -1 : static_cast<int>(pos);
        }

        String substring(unsigned int start) const {
            if (start >= s.size()) return String("");
            return String(s.substr(start));
        }

        String substring(unsigned int start, unsigned int end) const {
            if (start >= s.size()) return String("");
            if (end > s.size()) end = static_cast<unsigned int>(s.size());
            if (start >= end) return String("");
            return String(s.substr(start, end - start));
        }

        void toUpperCase() {
            std::transform(s.begin(), s.end(), s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        }

        bool equalsIgnoreCase(const char* other) const {
            if (!other) return s.empty();
            std::string a = s, b(other);
            std::transform(a.begin(), a.end(), a.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::transform(b.begin(), b.end(), b.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return a == b;
        }

        bool endsWith(const char* suffix) const {
            if (!suffix) return true;
            std::string suf(suffix);
            if (suf.size() > s.size()) return false;
            return s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
        }

        const char* c_str() const { return s.c_str(); }
    };

    struct SerialStub {
        void println(const char* msg) const { (void)msg; }
        void print(const char* msg) const { (void)msg; }
        void println(float v, int) const { (void)v; }
        void print(float v, int) const { (void)v; }
    };

    static SerialStub Serial;

    #ifdef PID_DEBUG_HOST_WITH_LOG
    #include "debug_log.h"
    #endif

#else
    // Khi chạy trên ESP32: dùng Arduino thực
    #include <Arduino.h>   // String, Serial, F() — chuẩn Arduino
#endif


// ================================================================
//  SECTION 1 — BASELINE PID CONSTANTS  (Physics-Derived)
// ================================================================
//
//  ┌─── LUẬN GIẢI VẬT LÝ ──────────────────────────────────────┐
//  │                                                              │
//  │  THRUST-TO-WEIGHT:                                          │
//  │   Motor 720 coreless @ 4.2V / 55mm / 2-blade               │
//  │   Thrust ước lượng: ~25-30g / motor                        │
//  │   Tổng thrust: ~100-120g  |  AUW: 45g                      │
//  │   TWR ≈ 2.5:1 → RẤT CAO → hệ thống cực nhạy cảm          │
//  │   → P_rate PHẢI thấp để tránh dao động biên độ lớn         │
//  │                                                              │
//  │  MOMENT QUÁN TÍNH (ước lượng, m_motor ≈ 4g/cái):          │
//  │   I_roll  = 2 × 0.004 × (0.035)²  ≈ 9.8 × 10⁻⁶ kg·m²    │
//  │   I_pitch = 2 × 0.004 × (0.0325)² ≈ 8.4 × 10⁻⁶ kg·m²    │
//  │                                                              │
//  │  PHÂN TÍCH ĐÒN BẨY:                                        │
//  │   α = ΔF × L / I  →  α ∝ ΔF / (m × L)                    │
//  │   Pitch arm 32.5mm < Roll arm 35mm                          │
//  │   → α_pitch > α_roll (Pitch phản ứng nhanh hơn)            │
//  │   → P_rate_PITCH < P_rate_ROLL                              │
//  │                                                              │
//  │  KHUNG PETG:                                                │
//  │   Nhựa dẻo → cộng hưởng tần số cao → nhiễu gyro lớn       │
//  │   D_rate = 0.0 → triệt khuếch đại nhiễu, bảo vệ motor      │
//  │   chổi than khỏi quá nhiệt do xung PWM liên tục            │
//  │                                                              │
//  │  THANG ĐO OUTPUT:                                           │
//  │   PWM LEDC 10-bit  (0 – 1023)                               │
//  │   PID Output clamp ±250  (~24% dải, đủ hiệu chỉnh)        │
//  └──────────────────────────────────────────────────────────────┘

//  ── ROLL  (arm = 35 mm | I ≈ 9.8e-6 kg·m² | phản ứng chậm hơn) ──
constexpr float ROLL_ANGLE_P  = 3.50f;   // Angle → Rate  [°/s per °]
constexpr float ROLL_RATE_P   = 1.00f;   // Rate P        [PWM per °/s]
constexpr float ROLL_RATE_I   = 0.05f;   // Rate I
constexpr float ROLL_RATE_D   = 0.00f;   // ← GIỮ = 0, PETG resonance

//  ── PITCH (arm = 32.5mm | I ≈ 8.4e-6 kg·m² | phản ứng nhanh hơn) ──
constexpr float PITCH_ANGLE_P = 3.50f;
constexpr float PITCH_RATE_P  = 0.85f;   // ← Thấp hơn ROLL, tránh rung Pitch
constexpr float PITCH_RATE_I  = 0.05f;
constexpr float PITCH_RATE_D  = 0.00f;

//  ── YAW   (Rate-only | không có Angle Loop) ──────────────────────
constexpr float YAW_RATE_P    = 1.20f;
constexpr float YAW_RATE_I    = 0.08f;
constexpr float YAW_RATE_D    = 0.00f;

//  ── Giới hạn chung ────────────────────────────────────────────────
constexpr float MAX_ANGLE_RATE   = 200.0f;   // °/s  — trần target_rate Angle Loop
constexpr float MAX_YAW_RATE     = 360.0f;   // °/s  — trần setpoint_rate Rate-only (Yaw)
constexpr float INTEGRAL_LIMIT   = 100.0f;   // Anti-windup clamp (±)
constexpr float PID_OUTPUT_LIMIT = 250.0f;   // Output final clamp (±250 / 1023)

//  ── Giới hạn tune qua Serial/Dabble ─────────────────────────────
constexpr float TUNE_P_ANGLE_MAX = 20.0f;
constexpr float TUNE_P_RATE_MAX  = 10.0f;
constexpr float TUNE_I_RATE_MAX  = 2.0f;
constexpr float TUNE_D_RATE_MAX  = 1.0f;


// ================================================================
//  SECTION 2 — CLASS CascadedPID
// ================================================================

class CascadedPID {
public:
    // ── Hệ số công khai — ghi trực tiếp từ parseTuneCommand ──
    float P_angle;
    float P_rate, I_rate, D_rate;

    // ── Constructor ───────────────────────────────────────────
    CascadedPID(float pa, float pr, float ir, float dr)
        : P_angle(pa), P_rate(pr), I_rate(ir), D_rate(dr),
          _integral(0.0f), _prevGyro(0.0f), _dReady(false) {}

    // ── Reset state ─────────────────────────────────────────────
    //  reset()           — full (disarm / va chạm)
    //  resetIntegral()   — chỉ I-state (đổi I gain)
    //  resetDerivative() — chỉ D-state (đổi D gain)
    void reset() {
#ifdef PID_DEBUG_HOST
        {
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "{\"D_rate\":%.6f,\"prevGyro\":%.2f,\"dReadyBefore\":%s}",
                     D_rate, _prevGyro, _dReady ? "true" : "false");
            if constexpr (false) { }  // Placeholder for debugLog
        }
#endif
        resetIntegral();
        resetDerivative();
    }

    void resetIntegral() {
        _integral = 0.0f;
    }

    void resetDerivative() {
        _prevGyro = 0.0f;
        _dReady   = false;
    }

    // ─────────────────────────────────────────────────────────
    //  compute()  —  Full Cascaded: Angle Loop → Rate Loop
    //  Dùng cho ROLL và PITCH
    // ─────────────────────────────────────────────────────────
    //  @param setpoint_angle  Góc mong muốn   [°, wrapped ±180°]
    //  @param measured_angle  Góc từ AHRS     [°, wrapped ±180°]
    //  @param measured_gyro   Vận tốc góc IMU [°/s]
    //  @param dt              Chu kỳ vòng lặp [s]  (0.001 @ 1 kHz)
    //  @return                PID output [-250 … +250]
    float compute(float setpoint_angle,
                  float measured_angle,
                  float measured_gyro,
                  float dt)
    {
        // FIX BUG 9: Angle wrapping — tính error với wraparound ±180°
        float angle_error = setpoint_angle - measured_angle;
        
        // Normalize angle_error to [-180, +180]
        while (angle_error > 180.0f)  angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;
        
        float target_rate = P_angle * angle_error;
        target_rate = _clamp(target_rate, -MAX_ANGLE_RATE, MAX_ANGLE_RATE);
        return _rateLoop(target_rate, measured_gyro, dt);
    }

    // ─────────────────────────────────────────────────────────
    //  computeRateOnly()  —  Rate Loop trực tiếp (không qua Angle Loop)
    //  Dùng cho YAW
    // ─────────────────────────────────────────────────────────
    //  @param setpoint_rate  Vận tốc góc mong muốn [°/s]
    //  @param measured_gyro  Vận tốc góc IMU        [°/s]
    //  @param dt             Chu kỳ vòng lặp         [s]
    //  @return               PID output [-250 … +250]
    float computeRateOnly(float setpoint_rate,
                          float measured_gyro,
                          float dt)
    {
        // FIX BUG 8: Clamp setpoint_rate để consistent với compute()
        setpoint_rate = _clamp(setpoint_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
        return _rateLoop(setpoint_rate, measured_gyro, dt);
    }

private:
    float _integral;
    float _prevGyro;   // Dùng cho D on measurement (không phải D on error)
    bool  _dReady;     // false sau reset/construct → bỏ qua D frame đầu

    float _rateLoop(float target_rate, float measured_gyro, float dt)
    {
        float rate_error = target_rate - measured_gyro;

        float P_term = P_rate * rate_error;

        _integral += I_rate * rate_error * dt;
        _integral  = _clamp(_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        // D on MEASUREMENT — bỏ qua frame đầu sau reset/construct để tránh spike
        float D_term = 0.0f;
        if (dt > 1e-6f) {
            if (_dReady) {
                D_term = -D_rate * (measured_gyro - _prevGyro) / dt;
            }
            _prevGyro = measured_gyro;
            _dReady   = true;
        }

        float output = P_term + _integral + D_term;
#ifdef PID_DEBUG_HOST
        if (D_rate > 0.0f) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "{\"dReady\":%s,\"D_rate\":%.6f,\"D_term\":%.2f,\"output\":%.2f,"
                     "\"measured_gyro\":%.2f,\"prevGyroBeforeUpdate\":%.2f}",
                     _dReady ? "true" : "false", D_rate, D_term, output, measured_gyro,
                     _prevGyro);
            if constexpr (false) { }  // Placeholder for debugLog
        }
#endif
        return _clamp(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    }

    static inline float _clamp(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }
};


// ================================================================
//  SECTION 3 — GLOBAL PID INSTANCES
// ================================================================

inline CascadedPID pidRoll (ROLL_ANGLE_P,  ROLL_RATE_P,  ROLL_RATE_I,  ROLL_RATE_D);
inline CascadedPID pidPitch(PITCH_ANGLE_P, PITCH_RATE_P, PITCH_RATE_I, PITCH_RATE_D);
inline CascadedPID pidYaw  (0.0f,          YAW_RATE_P,   YAW_RATE_I,   YAW_RATE_D);


// ================================================================
//  SECTION 4 — HELPER FUNCTIONS FOR COMMAND PARSING
// ================================================================

static bool parseFloatStrict(const String& valStr, float& out)
{
    if (valStr.length() == 0) return false;

    const char* start = valStr.c_str();
    char* end = nullptr;
    float v = strtof(start, &end);

    if (end == start) return false;

    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return false;

    out = v;
    return true;
}

static bool validateTuneValue(const String& key, float value)
{
    if (key.endsWith("_PA")) {
        if (value < 0.0f || value > TUNE_P_ANGLE_MAX) return false;
        return true;
    }
    if (key.endsWith("_PR")) {
        if (value < 0.0f || value > TUNE_P_RATE_MAX) return false;
        return true;
    }
    if (key.endsWith("_IR")) {
        if (value < 0.0f || value > TUNE_I_RATE_MAX) return false;
        return true;
    }
    if (key.endsWith("_DR")) {
        if (value < 0.0f || value > TUNE_D_RATE_MAX) return false;
        return true;
    }
    return true;
}


// ================================================================
//  SECTION 5 — MAIN TUNING COMMAND PARSER
// ================================================================

// FIX BUG 7: Pass String by const reference để tránh heap allocation
inline void parseTuneCommand(const String& cmd) {

    String cmdCopy = cmd;  // Single copy here, do the work locally
    cmdCopy.trim();
    if (cmdCopy.length() == 0) return;

    if (cmdCopy.equalsIgnoreCase("SHOW_PID")) {
        Serial.println(F("\r\n======== CURRENT PID PARAMS ========"));

        Serial.print(F("ROLL  | PA=")); Serial.print(pidRoll.P_angle, 4);
        Serial.print(F("  PR=")); Serial.print(pidRoll.P_rate, 4);
        Serial.print(F("  IR=")); Serial.print(pidRoll.I_rate, 4);
        Serial.print(F("  DR=")); Serial.println(pidRoll.D_rate, 4);

        Serial.print(F("PITCH | PA=")); Serial.print(pidPitch.P_angle, 4);
        Serial.print(F("  PR=")); Serial.print(pidPitch.P_rate, 4);
        Serial.print(F("  IR=")); Serial.print(pidPitch.I_rate, 4);
        Serial.print(F("  DR=")); Serial.println(pidPitch.D_rate, 4);

        Serial.print(F("YAW   |         PR=")); Serial.print(pidYaw.P_rate, 4);
        Serial.print(F("  IR=")); Serial.print(pidYaw.I_rate, 4);
        Serial.print(F("  DR=")); Serial.println(pidYaw.D_rate, 4);

        Serial.println(F("====================================\r\n"));
        return;
    }

    int spaceIdx = cmdCopy.indexOf(' ');
    if (spaceIdx < 1) {
        Serial.println(F("[TUNE] ERR: Sai cú pháp. VD: ROLL_PR 1.5 | SHOW_PID"));
        return;
    }

    String key = cmdCopy.substring(0, spaceIdx);
    key.toUpperCase();

    String valStr = cmdCopy.substring(spaceIdx + 1);
    valStr.trim();

#ifdef PID_DEBUG_HOST
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"key\":\"%s\",\"valStrLen\":%u,\"valStr\":\"%s\"}",
                 key.c_str(), valStr.length(), valStr.c_str());
        if constexpr (false) { }  // Placeholder for debugLog
    }
#endif

    if (valStr.length() == 0) {
        Serial.println(F("[TUNE] ERR: Thiếu giá trị. VD: ROLL_PR 1.5"));
        return;
    }

    float value = 0.0f;
    if (!parseFloatStrict(valStr, value)) {
        Serial.print(F("[TUNE] ERR: Giá trị không hợp lệ → \""));
        Serial.print(valStr);
        Serial.println(F("\""));
        return;
    }

    if (!validateTuneValue(key, value)) {
        Serial.print(F("[TUNE] ERR: Giá trị ngoài phạm vi cho "));
        Serial.println(key);
        return;
    }

    bool ok = true;

    if      (key == "ROLL_PA")  { pidRoll.P_angle = value; }
    else if (key == "ROLL_PR")  { pidRoll.P_rate  = value; }
    else if (key == "ROLL_IR")  { pidRoll.I_rate  = value; pidRoll.resetIntegral(); }
    else if (key == "ROLL_DR")  { pidRoll.D_rate  = value; pidRoll.resetDerivative(); }

    else if (key == "PITCH_PA") { pidPitch.P_angle = value; }
    else if (key == "PITCH_PR") { pidPitch.P_rate  = value; }
    else if (key == "PITCH_IR") { pidPitch.I_rate  = value; pidPitch.resetIntegral(); }
    else if (key == "PITCH_DR") { pidPitch.D_rate  = value; pidPitch.resetDerivative(); }

    else if (key == "YAW_PR")   { pidYaw.P_rate   = value; }
    else if (key == "YAW_IR")   { pidYaw.I_rate   = value; pidYaw.resetIntegral(); }
    else if (key == "YAW_DR")   { pidYaw.D_rate   = value; pidYaw.resetDerivative(); }
    else if (key == "YAW_PA") {
        Serial.println(F("[TUNE] WARN: YAW không có Angle Loop → YAW_PA bị từ chối."));
        ok = false;
    }

    else {
        Serial.print(F("[TUNE] ERR: Lệnh không hợp lệ → "));
        Serial.println(key);
        ok = false;
    }

    if (ok) {
        Serial.print(F("[TUNE] OK  "));
        Serial.print(key);
        Serial.print(F(" = "));
        Serial.println(value, 4);
    }
}


// ================================================================
//  USAGE NOTES (cho file .ino chính)
// ================================================================
//
//  #include "pid_core.h"           // header — có thể include từ nhiều TU
//
//  // Trong loop() @ 1 kHz (dt = 0.001f):
//  float rollOut  = pidRoll.compute (rcRoll,  ahrsRoll,  gyroX, dt);
//  float pitchOut = pidPitch.compute(rcPitch, ahrsPitch, gyroY, dt);
//  float yawOut   = pidYaw.computeRateOnly(rcYaw, gyroZ, dt);
//
//  // Tune qua Dabble Terminal:
//  if (DabbleTerminal.available()) {
//      parseTuneCommand(DabbleTerminal.readString());
//  }
//
//  // Hoặc tune qua Serial (debug):
//  if (Serial.available()) {
//      parseTuneCommand(Serial.readStringUntil('\n'));
//  }
//
// ================================================================
