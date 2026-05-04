#pragma once
// ============================================================
//  nunchuk.h — Nunchuk I2C 驅動
//  支援搖桿、加速度計（含低通濾波）、C/Z 按鈕
//  相容 ESP32-C3 (Wire on any GPIO)
// ============================================================
#include <Wire.h>
#include <math.h>

#define NUNCHUK_ADDR 0x52

// ── 原始資料結構 ─────────────────────────────────────────────
struct NunchukRaw {
    uint8_t  joyX;      // 0~255，中心≈128
    uint8_t  joyY;      // 0~255，中心≈128
    int16_t  accelX;    // 0~1023，靜止≈512
    int16_t  accelY;    // 0~1023，靜止≈512
    int16_t  accelZ;    // 0~1023，靜止≈700（感受重力）
    bool     btnC;
    bool     btnZ;
};

// ── 處理後資料結構 ────────────────────────────────────────────
struct NunchukData {
    // 搖桿（帶死區處理後，-127~127）
    int8_t   joyX;
    int8_t   joyY;

    // 加速度計原始有號值（以 512 為零點，-512~511）
    int16_t  accelX;
    int16_t  accelY;
    int16_t  accelZ;

    // 低通濾波後的 g 值（浮點，適合傾斜計算）
    float    gX;
    float    gY;
    float    gZ;

    // 傾斜角（度，由濾波後 g 值計算）
    float    pitch;  // 前/後傾斜 (Y 軸旋轉)
    float    roll;   // 左/右傾斜 (X 軸旋轉)

    // 合力大小（≈1.0g 靜止，>2.0g 甩動）
    float    magnitude;

    // 狀態旗標
    bool     btnC;
    bool     btnZ;
    bool     isStill;    // 合力接近 1g → 靜止
    bool     isFlicked;  // 合力 > 2g → 甩動
};

// ── Nunchuk 驅動類別 ──────────────────────────────────────────
class Nunchuk {
public:
    // 低通濾波係數：0.0（完全平滑）~ 1.0（無濾波）
    // 建議值 0.1~0.3；越小延遲越大但越穩定
    float filterAlpha = 0.15f;

    // 死區大小（搖桿）
    uint8_t deadzone = 12;

    // ── 初始化 ───────────────────────────────────────────────
    bool begin(uint8_t sda = 6, uint8_t scl = 7) {
        Wire.begin(sda, scl);
        Wire.setClock(100000);  // 100 kHz（Nunchuk 不支援 400kHz）

        // 初始化序列：解除加密（非官方但通用）
        if (!writeReg(0xF0, 0x55)) return false;
        delay(10);
        if (!writeReg(0xFB, 0x00)) return false;
        delay(20);

        // 預熱：丟棄第一筆可能不完整的資料
        NunchukRaw dummy;
        readRaw(dummy);
        delay(20);

        // 初始化濾波器狀態
        _gX = 0.0f;
        _gY = 0.0f;
        _gZ = 1.0f;  // 初始假設 Z 朝上（重力方向）
        _ready = true;

        return true;
    }

    // ── 讀取並處理一幀資料 ───────────────────────────────────
    // 回傳 false 代表 I2C 通訊失敗
    bool read(NunchukData &out) {
        if (!_ready) return false;

        NunchukRaw raw;
        if (!readRaw(raw)) return false;

        // 搖桿：帶死區映射到 -127~127
        out.joyX = mapAxis(raw.joyX);
        out.joyY = -mapAxis(raw.joyY);  // Y 軸反轉：向上為正

        // 加速度計：以 512 為零點
        out.accelX = (int16_t)raw.accelX - 512;
        out.accelY = (int16_t)raw.accelY - 512;
        out.accelZ = (int16_t)raw.accelZ - 512;

        // 轉換為 g 值（±2g 量程，10-bit → 256 LSB/g）
        float gXraw = out.accelX / 256.0f;
        float gYraw = out.accelY / 256.0f;
        float gZraw = out.accelZ / 256.0f;

        // 低通濾波（指數移動平均）
        _gX = filterAlpha * gXraw + (1.0f - filterAlpha) * _gX;
        _gY = filterAlpha * gYraw + (1.0f - filterAlpha) * _gY;
        _gZ = filterAlpha * gZraw + (1.0f - filterAlpha) * _gZ;

        out.gX = _gX;
        out.gY = _gY;
        out.gZ = _gZ;

        // 傾斜角（atan2，單位：度）
        out.pitch = atan2f(_gY, _gZ) * (180.0f / M_PI);
        out.roll  = atan2f(_gX, _gZ) * (180.0f / M_PI);

        // 合力大小
        out.magnitude = sqrtf(_gX*_gX + _gY*_gY + _gZ*_gZ);

        // 狀態旗標
        out.btnC     = raw.btnC;
        out.btnZ     = raw.btnZ;
        out.isStill  = fabsf(out.magnitude - 1.0f) < 0.12f;
        out.isFlicked = out.magnitude > 2.0f;

        return true;
    }

private:
    bool  _ready = false;
    float _gX = 0.0f, _gY = 0.0f, _gZ = 1.0f;  // 濾波器狀態

    // ── 寫入單一暫存器 ────────────────────────────────────────
    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(NUNCHUK_ADDR);
        Wire.write(reg);
        Wire.write(val);
        return Wire.endTransmission() == 0;
    }

    // ── 讀取 6 bytes 原始封包 ─────────────────────────────────
    bool readRaw(NunchukRaw &raw) {
        // 送出讀取請求
        Wire.beginTransmission(NUNCHUK_ADDR);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;

        delayMicroseconds(600);

        if (Wire.requestFrom(NUNCHUK_ADDR, 6) != 6) return false;

        uint8_t b[6];
        for (int i = 0; i < 6; i++) b[i] = Wire.read();

        // Byte 0: 搖桿 X
        raw.joyX = b[0];
        // Byte 1: 搖桿 Y
        raw.joyY = b[1];
        // Bytes 2~4: 加速度計高 8 bits，Byte 5: 各軸低 2 bits
        raw.accelX = ((int16_t)b[2] << 2) | ((b[5] >> 2) & 0x03);
        raw.accelY = ((int16_t)b[3] << 2) | ((b[5] >> 4) & 0x03);
        raw.accelZ = ((int16_t)b[4] << 2) | ((b[5] >> 6) & 0x03);
        // Byte 5 bit 0: Z 按鈕（0=按下），bit 1: C 按鈕
        raw.btnZ   = !(b[5] & 0x01);
        raw.btnC   = !((b[5] >> 1) & 0x01);

        return true;
    }

    // ── 搖桿軸映射（帶死區）──────────────────────────────────
    int8_t mapAxis(uint8_t raw) {
        int val = (int)raw - 128;
        if (abs(val) < deadzone) return 0;
        if (val > 0) return (int8_t)((val - deadzone) * 127 / (127 - deadzone));
        else         return (int8_t)((val + deadzone) * 127 / (127 - deadzone));
    }
};