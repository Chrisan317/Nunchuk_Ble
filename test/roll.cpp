// ============================================================
//  main.cpp — Nunchuk BLE 複合 HID（Gamepad + 體感滑鼠）
//  ESP32-C3 SuperMini + PlatformIO + NimBLE-Arduino
//
//  接線:
//    Nunchuk VCC → 3.3V      Nunchuk GND → GND
//    Nunchuk SDA → GPIO 6    Nunchuk SCL → GPIO 7
//
//  切換方式：長按 C + Z 同時 2 秒
//    → Gamepad 模式：LED 閃 1 下
//    → Mouse   模式：LED 閃 2 下
//
//  Mouse 模式操作：
//    傾斜 Nunchuk → 移動游標（pitch/roll → deltaY/deltaX）
//    搖桿 Y 軸    → 滾輪上下捲動
//    C 按鈕       → 左鍵
//    Z 按鈕       → 右鍵
// ============================================================

#include <Arduino.h>
#include "nunchuk.h"
#include "ble_hid.h"

// ── 硬體腳位 ─────────────────────────────────────────────────
constexpr uint8_t SDA_PIN = 6;
constexpr uint8_t SCL_PIN = 7;
constexpr uint8_t LED_PIN = 8;  // ESP32-C3 SuperMini 內建 LED（低電位亮）

// ── 模式定義 ─────────────────────────────────────────────────
enum class Mode { GAMEPAD, MOUSE };

// ── 更新頻率 ─────────────────────────────────────────────────
constexpr uint32_t UPDATE_MS      = 16;    // 60 Hz 主循環
constexpr uint32_t IDLE_MS        = 50;    // 省電降頻 20 Hz
constexpr uint32_t STILL_TIMEOUT  = 5000;  // 靜止幾毫秒後降頻

// ── 切換設定 ─────────────────────────────────────────────────
constexpr uint32_t SWITCH_HOLD_MS = 2000;  // 長按門檻（毫秒）

// ── 體感滑鼠靈敏度（中等）────────────────────────────────────
// 角度（度）乘以係數後得到每幀 delta pixel
// 中等：係數 1.8，死角 ±3°
constexpr float    MOUSE_SENSITIVITY = 1.8f;
constexpr float    MOUSE_DEADANGLE   = 3.0f;   // 度，小於此角度視為靜止
constexpr int8_t   MOUSE_MAX_DELTA   = 20;      // 每幀最大移動量（pixel）
constexpr float    SCROLL_SENSITIVITY = 0.15f;  // 搖桿 Y → 滾輪係數

// ── 甩動偵測 ─────────────────────────────────────────────────
constexpr int   FLICK_FRAMES    = 2;
constexpr float FLICK_THRESHOLD = 2.0f;

// ── 全域物件 ─────────────────────────────────────────────────
Nunchuk       nunchuk;
BLEHIDGamepad gamepad;
Mode          currentMode = Mode::GAMEPAD;

// ── 狀態結構 ─────────────────────────────────────────────────
struct AppState {
    // 時間
    uint32_t lastUpdateMs  = 0;
    uint32_t lastMotionMs  = 0;
    bool     idleMode      = false;

    // 輸入快取（省電用）
    int8_t   lastJoyX = 0, lastJoyY = 0;
    bool     lastBtnC = false, lastBtnZ = false;

    // 甩動
    int      flickFrames = 0;

    // 模式切換：長按 C+Z
    uint32_t bothPressedMs = 0;   // C+Z 同時按下的起始時間
    bool     bothWasDown   = false;
    bool     switchTriggered = false; // 這次長按是否已觸發，避免重複觸發
} st;

// ── 除錯輸出開關 ─────────────────────────────────────────────
#define DEBUG_SERIAL 1
#define DEBUG_ACCEL  0

// ════════════════════════════════════════════════════════════
//  LED 輔助函式
// ════════════════════════════════════════════════════════════

// 低電位亮燈
inline void ledOn()  { digitalWrite(LED_PIN, LOW);  }
inline void ledOff() { digitalWrite(LED_PIN, HIGH); }

// 阻塞式閃爍 N 下（每下 on 150ms / off 200ms，最後留 300ms 間隔）
void ledBlink(int times) {
    for (int i = 0; i < times; i++) {
        ledOn();  delay(150);
        ledOff(); delay(200);
    }
    delay(300);
}

// ════════════════════════════════════════════════════════════
//  角度 → 游標 delta 轉換
// ════════════════════════════════════════════════════════════
int8_t angleToDelta(float angle) {
    if (fabsf(angle) < MOUSE_DEADANGLE) return 0;

    // 超過死角的部分才計算，保留符號
    float effective = angle - (angle > 0 ? MOUSE_DEADANGLE : -MOUSE_DEADANGLE);
    float delta = effective * MOUSE_SENSITIVITY;
    return (int8_t)constrain((int)delta, -MOUSE_MAX_DELTA, MOUSE_MAX_DELTA);
}

// ════════════════════════════════════════════════════════════
//  切換模式偵測（長按 C+Z 2 秒）
//  回傳 true 代表這幀剛觸發切換
// ════════════════════════════════════════════════════════════
bool checkModeSwitch(const NunchukData &d, uint32_t now) {
    bool bothDown = d.btnC && d.btnZ;

    if (bothDown) {
        if (!st.bothWasDown) {
            // 剛開始按下，記錄時間
            st.bothPressedMs  = now;
            st.switchTriggered = false;
        } else if (!st.switchTriggered &&
                   (now - st.bothPressedMs) >= SWITCH_HOLD_MS) {
            // 達到門檻，觸發切換
            st.switchTriggered = true;
            st.bothWasDown     = true;
            return true;
        }
    } else {
        st.switchTriggered = false;
    }

    st.bothWasDown = bothDown;
    return false;
}

// ════════════════════════════════════════════════════════════
//  省電判斷
// ════════════════════════════════════════════════════════════
bool hasInputChanged(const NunchukData &d) {
    return (d.joyX != st.lastJoyX || d.joyY != st.lastJoyY ||
            d.btnC != st.lastBtnC || d.btnZ != st.lastBtnZ ||
            !d.isStill);
}

void cacheInput(const NunchukData &d) {
    st.lastJoyX = d.joyX;  st.lastJoyY = d.joyY;
    st.lastBtnC = d.btnC;  st.lastBtnZ = d.btnZ;
}

// ════════════════════════════════════════════════════════════
//  甩動偵測
// ════════════════════════════════════════════════════════════
bool detectFlick(const NunchukData &d) {
    if (d.magnitude > FLICK_THRESHOLD) st.flickFrames++;
    else                               st.flickFrames = 0;
    return st.flickFrames >= FLICK_FRAMES;
}

// ════════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(LED_PIN, OUTPUT);
    ledOff();

    Serial.println("============================================");
    Serial.println("  Nunchuk BLE HID (Gamepad + Mouse) 啟動");
    Serial.println("============================================");

    // 初始化 Nunchuk
    Serial.printf("[Nunchuk] 初始化 (SDA=%d SCL=%d)...\n", SDA_PIN, SCL_PIN);
    if (!nunchuk.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("[Nunchuk] ❌ 失敗，請檢查接線");
        // 快速閃爍代表硬體錯誤
        while (true) { ledOn(); delay(100); ledOff(); delay(100); }
    }
    nunchuk.filterAlpha = 0.15f;
    nunchuk.deadzone    = 12;
    Serial.println("[Nunchuk] ✓ 就緒");

    // 初始化 BLE
    Serial.println("[BLE] 初始化...");
    gamepad.begin("Nunchuk BLE");
    Serial.println("[BLE] ✓ 廣播中，請配對「Nunchuk BLE」");

    // 啟動提示：閃 1 下 = Gamepad 模式
    delay(500);
    ledBlink(1);

    st.lastUpdateMs = millis();
    st.lastMotionMs = millis();

    Serial.println("============================================");
    Serial.println("  初始模式：Gamepad");
    Serial.println("  長按 C+Z 2 秒切換模式");
    Serial.println("============================================\n");
}

// ════════════════════════════════════════════════════════════
//  Loop
// ════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // 動態更新頻率
    uint32_t interval = st.idleMode ? IDLE_MS : UPDATE_MS;
    if ((now - st.lastUpdateMs) < interval) return;
    st.lastUpdateMs = now;

    // 讀取 Nunchuk
    NunchukData d;
    if (!nunchuk.read(d)) {
        Serial.println("[Nunchuk] ⚠ 讀取失敗");
        return;
    }

    // ── 切換模式偵測 ────────────────────────────────────────
    if (checkModeSwitch(d, now)) {
        // 切換
        if (currentMode == Mode::GAMEPAD) {
            currentMode = Mode::MOUSE;
            Serial.println("[模式] 切換 → 體感滑鼠");
            ledBlink(2);  // 2 下 = Mouse 模式
        } else {
            currentMode = Mode::GAMEPAD;
            Serial.println("[模式] 切換 → Gamepad");
            ledBlink(1);  // 1 下 = Gamepad 模式
        }
        // 切換後跳過這幀避免誤觸
        cacheInput(d);
        return;
    }

    // ── 甩動偵測（僅 Gamepad 模式輸出到序列埠）─────────────
    if (currentMode == Mode::GAMEPAD && detectFlick(d)) {
        Serial.printf("[甩動] %.2fg  pitch=%.1f°  roll=%.1f°\n",
                      d.magnitude, d.pitch, d.roll);
    }

    // ── 省電：靜止降頻 ──────────────────────────────────────
    if (hasInputChanged(d)) {
        st.lastMotionMs = now;
        st.idleMode     = false;
    } else if ((now - st.lastMotionMs) > STILL_TIMEOUT) {
        if (!st.idleMode) {
            st.idleMode = true;
            Serial.println("[省電] 靜止降頻");
        }
    }

    // ── 傳送 HID Report ──────────────────────────────────────
    if (gamepad.connected) {

        if (currentMode == Mode::GAMEPAD) {
            // ── Gamepad：搖桿 + 加速度計 + 按鈕 ────────────
            gamepad.sendGamepadReport(
                d.joyX,  d.joyY,
                d.accelX, d.accelY, d.accelZ,
                d.btnC,  d.btnZ
            );

        } else {
            // ── Mouse：傾斜 → 游標，搖桿 Y → 滾輪 ──────────
            //
            // pitch > 0：Nunchuk 前傾 → 游標向上（deltaY 為負）
            // roll  > 0：Nunchuk 右傾 → 游標向右（deltaX 為正）
            int8_t deltaX = angleToDelta(d.roll);
            int8_t deltaY = -angleToDelta(d.pitch);  // 螢幕 Y 軸向下為正，故取反

            // 搖桿 Y 對應滾輪（向上推搖桿 = 向上捲動）
            int8_t wheel = 0;
            if (abs(d.joyY) > 10) {  // 搖桿死區
                wheel = (int8_t)constrain(
                    (int)(d.joyY * SCROLL_SENSITIVITY), -8, 8);
            }

            // 切換模式時 C+Z 同時按住，避免誤觸左右鍵
            // 只要 bothWasDown 仍為 true（還沒放開）就不送按鍵
            bool sendButtons = !st.bothWasDown;

            gamepad.sendMouseReport(
                deltaX, deltaY, wheel,
                sendButtons && d.btnC,
                sendButtons && d.btnZ
            );
        }
    }

    cacheInput(d);

    // ── 除錯輸出 ─────────────────────────────────────────────
#if DEBUG_SERIAL
    static uint32_t lastPrintMs = 0;
    if ((now - lastPrintMs) >= 250) {
        lastPrintMs = now;
        const char* modeStr = (currentMode == Mode::GAMEPAD) ? "GAMEPAD" : "MOUSE  ";
        Serial.printf("[%s] Joy(%4d,%4d)  Pitch=%5.1f°  Roll=%5.1f°  "
                      "|g|=%.2f  C=%d Z=%d  BLE=%d\n",
                      modeStr, d.joyX, d.joyY,
                      d.pitch, d.roll, d.magnitude,
                      d.btnC, d.btnZ, gamepad.connected);
#if DEBUG_ACCEL
        Serial.printf("          gX=%5.3f  gY=%5.3f  gZ=%5.3f\n",
                      d.gX, d.gY, d.gZ);
#endif
    }
#endif
}