// ============================================================
//  main.cpp - Nunchuk BLE Composite HID (Gamepad + Mouse)
//  XIAO ESP32-C3 + PlatformIO + NimBLE-Arduino
//
//  Wiring:
//    Nunchuk VCC -> 3.3V        Nunchuk GND -> GND
//    Nunchuk SDA -> GPIO 6      Nunchuk SCL -> GPIO 7
//    Power button  -> GPIO 3     (internal pull-up, press to GND)
//    Battery ADC   -> GPIO 2/A0  (2:1 voltage divider)
//    P-MOSFET gate -> GPIO 4     (LOW = Nunchuk on, HIGH = off)
//    P-MOSFET: Source -> 3.3V  Drain -> Nunchuk VCC
//              100kΩ pull-up Gate-to-Source (default off when GPIO floats)
//
//  LED (GPIO 10, active HIGH):
//    Normal      : heartbeat blink every 2s
//    Pre-sleep   : 2 blinks then fade out
//    Wake confirm: 2 fast blinks after hold confirmed
//
//  Controls:
//    Hold C+Z 2s       -> toggle Gamepad / Mouse mode (saved to NVS)
//    Hold power 2s     -> deep sleep
//    Hold power 2s     -> wake from deep sleep
//                         (release early = back to sleep, no LED)
//    Idle 10 min       -> auto deep sleep (connected or not)
//
//  Mouse mode:
//    Joystick X/Y      -> cursor movement
//    C button          -> left click
//    Z button          -> right click
//    Hold C alone 1.5s -> cycle sensitivity Low/Mid/High (saved to NVS)
//    Swipe right       -> next page  (Consumer: Scan Next Track)
//    Swipe left        -> prev page  (Consumer: Scan Prev Track)
//
//  Swipe gesture (Mouse mode only):
//    Detects X-axis acceleration delta (change per frame, not absolute)
//    so gravity has no effect on detection.
//    Three-layer false-positive protection:
//      1. Delta threshold  : |deltaGX| must exceed SWIPE_DELTA_THRESH
//      2. Frame confirming : N consecutive frames in same direction
//      3. Cooldown         : SWIPE_COOLDOWN_MS after each trigger
//
//  Accelerometer reserved features (uncomment to enable):
//    Flick (magnitude > 2g) -> middle click
//    Still 3s               -> freeze cursor to prevent drift
//    Flip (gZ < -0.7)       -> pause all input
// ============================================================

#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <Preferences.h>
#include "nunchuk.h"
#include "ble_hid.h"

// ── Pin assignments ───────────────────────────────────────────
constexpr uint8_t SDA_PIN         = 6;
constexpr uint8_t SCL_PIN         = 7;
constexpr uint8_t LED_PIN         = 10;  // Active HIGH
constexpr uint8_t POWER_PIN       = 3;   // Power button (internal pull-up)
constexpr uint8_t BAT_PIN         = 2;   // Battery ADC (A0, 2:1 divider)
constexpr uint8_t NUNCHUK_PWR_PIN = 4;   // P-MOSFET gate (LOW=on, HIGH=off)

// ── Mode ──────────────────────────────────────────────────────
enum class Mode { GAMEPAD, MOUSE };

// ── Mouse sensitivity presets ─────────────────────────────────
struct SensConfig {
    float  scale;     // Joystick value (-127~127) * scale = delta pixels
    int8_t maxDelta;  // Max cursor pixels per frame
};
constexpr SensConfig SENS_TABLE[3] = {
    { 0.10f, 12 },   // Low  - precise control
    { 0.18f, 20 },   // Mid  - balanced (default)
    { 0.28f, 30 },   // High - fast, suits large displays
};

// ── Timing thresholds (ms) ────────────────────────────────────
constexpr uint32_t UPDATE_MS      = 16;       // Main loop ~60 Hz
constexpr uint32_t IDLE_MS        = 50;       // Power-save ~20 Hz
constexpr uint32_t STILL_TIMEOUT  = 5000;     // Throttle after 5s no movement
constexpr uint32_t SWITCH_HOLD_MS = 2000;     // Hold C+Z to switch mode
constexpr uint32_t SLEEP_HOLD_MS  = 2000;     // Hold power button to sleep
constexpr uint32_t SLEEP_WARN_MS  = 1500;     // LED warning threshold before sleep
constexpr uint32_t SENS_HOLD_MS   = 1500;     // Hold C to change sensitivity
constexpr uint32_t AUTO_SLEEP_MS  = 600000;   // Auto-sleep after idle (10 min)
constexpr uint32_t BAT_UPDATE_MS  = 60000;    // Battery report interval (60s)
constexpr uint32_t LED_BLINK_MS   = 2000;     // Heartbeat interval
constexpr uint32_t WAKE_HOLD_MS   = 2000;     // Required hold time to confirm wake

// ── Swipe gesture settings ────────────────────────────────────
// Detects change in X-axis g-value (deltaGX) per frame.
// Using delta instead of absolute value removes gravity bias.
constexpr float    SWIPE_DELTA_THRESH = 0.18f;  // Min |deltaGX| per frame to count
constexpr int      SWIPE_CONFIRM_FRAMES = 3;    // Consecutive frames to confirm swipe
constexpr uint32_t SWIPE_COOLDOWN_MS  = 800;    // Min ms between swipe triggers

// ── Reserved accelerometer constants (uncomment to enable) ────
// constexpr float    FLICK_THRESHOLD = 2.0f;   // Flick magnitude (g)
// constexpr int      FLICK_FRAMES    = 2;       // Consecutive frames to confirm flick
// constexpr uint32_t STILL_LOCK_MS   = 3000;   // Still duration to lock cursor
// constexpr float    FLIP_THRESHOLD  = -0.7f;  // gZ threshold for flip-pause

// ── NVS keys ──────────────────────────────────────────────────
constexpr char NVS_NS[]   = "nunchuk";
constexpr char NVS_SENS[] = "sens";  // Sensitivity index: 0=Low 1=Mid 2=High
constexpr char NVS_MODE[] = "mode";  // Last mode: 0=Gamepad 1=Mouse

// ── Global objects ────────────────────────────────────────────
Nunchuk       nunchuk;
BLEHIDGamepad gamepad;
Preferences   prefs;
Mode          currentMode = Mode::GAMEPAD;
uint8_t       sensIdx     = 1;  // Default: Mid

// ── Runtime state ─────────────────────────────────────────────
struct AppState {
    // Timing
    uint32_t lastUpdateMs   = 0;
    uint32_t lastMotionMs   = 0;  // Last frame with any joystick/button activity
    uint32_t lastBatMs      = 0;
    uint32_t lastLedMs      = 0;
    bool     idleMode       = false;

    // Input cache for change detection
    int8_t   lastJoyX = 0, lastJoyY = 0;
    bool     lastBtnC = false, lastBtnZ = false;

    // Mode switch: hold C+Z
    uint32_t bothPressedMs   = 0;
    bool     bothWasDown     = false;
    bool     switchTriggered = false;

    // Power button: hold to sleep
    uint32_t powerPressedMs  = 0;
    bool     powerWasDown    = false;
    bool     sleepWarned     = false;

    // Sensitivity switch: hold C alone
    uint32_t cPressedMs      = 0;
    bool     cWasDown        = false;
    bool     sensTriggered   = false;

    // Swipe gesture
    float    prevGX          = 0.0f;  // Previous frame filtered gX
    int      swipeFrames     = 0;     // Consecutive frames in current direction
    int      swipeDir        = 0;     // +1 = right, -1 = left, 0 = none
    uint32_t lastSwipeMs     = 0;     // Timestamp of last swipe trigger

    // Reserved accelerometer state (uncomment to enable)
    // int      flickFrames   = 0;     // Consecutive flick frames
    // uint32_t stillStartMs  = 0;     // Timestamp when stillness began
    // bool     cursorLocked  = false; // True when cursor is frozen
    // bool     flipped       = false; // True when controller is flipped
} st;

// ── Debug switch ──────────────────────────────────────────────
#define DEBUG_SERIAL 1

// ════════════════════════════════════════════════════════════
//  LED helpers (active HIGH)
// ════════════════════════════════════════════════════════════
inline void ledOn()  { digitalWrite(LED_PIN, HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LOW);  }

void ledBlink(int times, int onMs = 120, int offMs = 150) {
    for (int i = 0; i < times; i++) {
        ledOn();  delay(onMs);
        ledOff(); delay(offMs);
    }
}

// Non-blocking heartbeat: one 40ms blink every LED_BLINK_MS
void updateHeartbeat(uint32_t now) {
    if ((now - st.lastLedMs) >= LED_BLINK_MS) {
        st.lastLedMs = now;
        ledOn();  delay(40);
        ledOff();
    }
}

// ════════════════════════════════════════════════════════════
//  Battery
//  analogReadMilliVolts() applies Espressif ADC correction curve.
//  2:1 voltage divider halves battery voltage before ADC pin.
//  LiPo range: 3.0V (0%) to 4.2V (100%).
// ════════════════════════════════════════════════════════════
uint8_t readBatteryPercent() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogReadMilliVolts(BAT_PIN);
        delay(2);
    }
    float voltage = 2.0f * sum / 16 / 1000.0f;  // Undo divider, mV -> V
    float pct = (voltage - 3.0f) / (4.2f - 3.0f) * 100.0f;
    return (uint8_t)constrain((int)pct, 0, 100);
}

void updateBattery(uint32_t now) {
    if ((now - st.lastBatMs) < BAT_UPDATE_MS) return;
    st.lastBatMs = now;
    uint8_t pct = readBatteryPercent();
    gamepad.updateBattery(pct);
    Serial.printf("[BAT] %d%%\n", pct);
}

// ════════════════════════════════════════════════════════════
//  Deep sleep
//  Stops BLE, waits for button release, enters deep sleep.
//  GPIO 3 low-level wakeup (ESP32-C3 gpio wakeup API).
// ════════════════════════════════════════════════════════════
void goToSleep(const char* reason) {
    Serial.printf("[SLEEP] %s\n", reason);
    Serial.flush();

    // Warning: 2 blinks then LED fade out
    ledBlink(2, 120, 150);
    delay(200);
    for (int i = 4; i >= 0; i--) {
        ledOn();  delay(i * 15);
        ledOff(); delay((4 - i) * 15 + 15);
    }

    // Stop advertising and disconnect gracefully
    NimBLEDevice::getAdvertising()->stop();
    gamepad.shuttingDown = true;
    if (gamepad.connected) {
        NimBLEDevice::getServer()->disconnect(
            NimBLEDevice::getServer()->getPeerInfo(0).getAddress());
        uint32_t t = millis();
        while (gamepad.connected && (millis() - t) < 500) delay(10);
    }

    Wire.end();

    // Cut Nunchuk power: P-MOS gate HIGH = off
    digitalWrite(NUNCHUK_PWR_PIN, HIGH);
    gpio_hold_en((gpio_num_t)NUNCHUK_PWR_PIN);  // Latch HIGH through deep sleep
    gpio_deep_sleep_hold_en();                   // Enable global GPIO hold

    ledOff();

    // Wait for button release to prevent immediate re-wake
    while (digitalRead(POWER_PIN) == LOW) delay(10);
    delay(100);

    gpio_set_direction((gpio_num_t)POWER_PIN, GPIO_MODE_INPUT);
    esp_deep_sleep_enable_gpio_wakeup(1ULL << POWER_PIN,
                                      ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
    // Does not return - wake triggers a full reset
}

// ════════════════════════════════════════════════════════════
//  Wake hold check
//  After GPIO wakeup, requires WAKE_HOLD_MS of sustained press.
//  If button is released early, returns to sleep without LED.
// ════════════════════════════════════════════════════════════
void checkWakeHold() {
    uint32_t pressStart = millis();
    while (digitalRead(POWER_PIN) == LOW) {
        if ((millis() - pressStart) >= WAKE_HOLD_MS) {
            return;  // Hold confirmed, continue boot
        }
        delay(10);
    }
    // Released before threshold - sleep again silently, no LED
    Serial.println("[WAKE] Early release, returning to sleep");
    Serial.flush();
    gpio_set_direction((gpio_num_t)POWER_PIN, GPIO_MODE_INPUT);
    esp_deep_sleep_enable_gpio_wakeup(1ULL << POWER_PIN,
                                      ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
}

// ════════════════════════════════════════════════════════════
//  Swipe gesture detection (Mouse mode, X-axis)
//
//  Uses per-frame delta of filtered gX to avoid gravity bias.
//  Three protection layers:
//    1. SWIPE_DELTA_THRESH  : ignores slow drift
//    2. SWIPE_CONFIRM_FRAMES: requires N consecutive frames
//    3. SWIPE_COOLDOWN_MS   : prevents repeat triggers
//
//  Returns: +1 = swipe right (next page)
//           -1 = swipe left  (prev page)
//            0 = no gesture
// ════════════════════════════════════════════════════════════
int checkSwipe(const NunchukData& d, uint32_t now) {
    // Compute per-frame acceleration delta on X axis
    float deltaGX = d.gX - st.prevGX;
    st.prevGX = d.gX;

    // Determine direction of this frame (+1 / -1 / 0)
    int frameDir = 0;
    if      (deltaGX >  SWIPE_DELTA_THRESH) frameDir = +1;
    else if (deltaGX < -SWIPE_DELTA_THRESH) frameDir = -1;

    if (frameDir != 0 && frameDir == st.swipeDir) {
        // Same direction as accumulating swipe - increment counter
        st.swipeFrames++;
    } else {
        // Direction changed or below threshold - reset
        st.swipeFrames = (frameDir != 0) ? 1 : 0;
        st.swipeDir    = frameDir;
    }

    // Trigger when confirmed and cooldown has elapsed
    if (st.swipeFrames >= SWIPE_CONFIRM_FRAMES &&
        (now - st.lastSwipeMs) >= SWIPE_COOLDOWN_MS) {
        int triggered  = st.swipeDir;
        st.swipeFrames = 0;
        st.swipeDir    = 0;
        st.lastSwipeMs = now;
        return triggered;
    }

    return 0;
}

// ════════════════════════════════════════════════════════════
//  Joystick to cursor delta
// ════════════════════════════════════════════════════════════
int8_t joyToDelta(int8_t joy, const SensConfig& s) {
    return (int8_t)constrain((int)(joy * s.scale), -s.maxDelta, s.maxDelta);
}

// ════════════════════════════════════════════════════════════
//  Reserved accelerometer functions (uncomment body to enable)
// ════════════════════════════════════════════════════════════

// Flick detection -> middle click
// Returns true on the frame a flick is confirmed.
// Replace 'false' with 'detectFlick(d)' in sendMouseReport() to enable.
// bool detectFlick(const NunchukData& d) {
//     if (d.magnitude > FLICK_THRESHOLD) st.flickFrames++;
//     else                               st.flickFrames = 0;
//     bool hit = (st.flickFrames == FLICK_FRAMES);
//     if (hit) Serial.printf("[FLICK] %.2fg\n", d.magnitude);
//     return hit;
// }

// Still lock -> freeze cursor when controller is at rest
// Returns true when cursor movement should be suppressed.
// Usage: if (checkCursorLock(d, now)) { deltaX = 0; deltaY = 0; }
// bool checkCursorLock(const NunchukData& d, uint32_t now) {
//     if (!d.isStill) { st.stillStartMs = now; st.cursorLocked = false; return false; }
//     if ((now - st.stillStartMs) >= STILL_LOCK_MS) {
//         if (!st.cursorLocked) { st.cursorLocked = true; Serial.println("[LOCK] Cursor locked"); }
//         return true;
//     }
//     return false;
// }

// Flip pause -> pause all input when controller is upside-down
// Returns true while flipped.
// Usage: if (checkFlipped(d)) { cacheInput(d); return; }
// bool checkFlipped(const NunchukData& d) {
//     bool now = (d.gZ < FLIP_THRESHOLD);
//     if (now != st.flipped) {
//         st.flipped = now;
//         Serial.printf("[FLIP] %s\n", now ? "Paused" : "Resumed");
//     }
//     return st.flipped;
// }

// ════════════════════════════════════════════════════════════
//  Power button: hold SLEEP_HOLD_MS to enter deep sleep
// ════════════════════════════════════════════════════════════
void checkPowerButton(uint32_t now) {
    bool pressed = (digitalRead(POWER_PIN) == LOW);
    if (pressed) {
        if (!st.powerWasDown) {
            st.powerPressedMs = now;
            st.sleepWarned    = false;
        } else {
            uint32_t held = now - st.powerPressedMs;
            if (!st.sleepWarned && held >= SLEEP_WARN_MS) {
                st.sleepWarned = true;
                ledOn();  // Solid LED: sleep countdown active
                Serial.println("[PWR] Keep holding to sleep, release to cancel");
            }
            if (held >= SLEEP_HOLD_MS) goToSleep("Power button held");
        }
        st.powerWasDown = true;
    } else {
        if (st.powerWasDown && st.sleepWarned) {
            ledOff();
            Serial.println("[PWR] Sleep cancelled");
        }
        st.powerWasDown = false;
        st.sleepWarned  = false;
    }
}

// ════════════════════════════════════════════════════════════
//  Mode switch: hold C+Z for SWITCH_HOLD_MS
// ════════════════════════════════════════════════════════════
bool checkModeSwitch(const NunchukData& d, uint32_t now) {
    bool bothDown = d.btnC && d.btnZ;
    if (bothDown) {
        if (!st.bothWasDown) {
            st.bothPressedMs   = now;
            st.switchTriggered = false;
        } else if (!st.switchTriggered &&
                   (now - st.bothPressedMs) >= SWITCH_HOLD_MS) {
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
//  Sensitivity switch: hold C alone for SENS_HOLD_MS (Mouse mode only)
// ════════════════════════════════════════════════════════════
void checkSensSwitch(const NunchukData& d, uint32_t now) {
    if (currentMode != Mode::MOUSE) return;
    bool cOnly = d.btnC && !d.btnZ;
    if (cOnly) {
        if (!st.cWasDown) {
            st.cPressedMs    = now;
            st.sensTriggered = false;
        } else if (!st.sensTriggered &&
                   (now - st.cPressedMs) >= SENS_HOLD_MS) {
            st.sensTriggered = true;
            sensIdx = (sensIdx + 1) % 3;
            prefs.putUChar(NVS_SENS, sensIdx);
            const char* labels[] = { "Low", "Mid", "High" };
            Serial.printf("[SENS] -> %s\n", labels[sensIdx]);
            ledBlink((int)sensIdx + 1, 80, 100);  // 1/2/3 blinks = Low/Mid/High
        }
        st.cWasDown = true;
    } else {
        st.sensTriggered = false;
        st.cWasDown      = false;
    }
}

// ════════════════════════════════════════════════════════════
//  Input change detection (for power-save throttling)
// ════════════════════════════════════════════════════════════
// Returns true if joystick or buttons changed.
// Used for auto-sleep timer - excludes accelerometer so table
// vibration does not reset the idle countdown.
bool hasInputChanged(const NunchukData& d) {
    return (d.joyX != st.lastJoyX || d.joyY != st.lastJoyY ||
            d.btnC != st.lastBtnC || d.btnZ  != st.lastBtnZ);
}

// Returns true if there is any motion including accelerometer.
// Used for 20Hz power-save throttle only.
bool hasMotionForThrottle(const NunchukData& d) {
    return hasInputChanged(d) || !d.isStill;
}

void cacheInput(const NunchukData& d) {
    st.lastJoyX = d.joyX;  st.lastJoyY = d.joyY;
    st.lastBtnC = d.btnC;  st.lastBtnZ = d.btnZ;
}

// ════════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);

    // Release GPIO hold state latched during previous deep sleep
    // Must be called before driving any GPIO
    gpio_hold_dis((gpio_num_t)NUNCHUK_PWR_PIN);
    gpio_deep_sleep_hold_dis();

    pinMode(LED_PIN,         OUTPUT);
    pinMode(POWER_PIN,       INPUT_PULLUP);
    pinMode(BAT_PIN,         INPUT);
    pinMode(NUNCHUK_PWR_PIN, OUTPUT);
    digitalWrite(NUNCHUK_PWR_PIN, LOW);  // P-MOS: gate LOW = conducts = Nunchuk powered
    ledOff();

    auto wakeReason = esp_sleep_get_wakeup_cause();

    Serial.println("============================================");
    Serial.println("  Nunchuk BLE HID - booting");
    Serial.println("============================================");

    if (wakeReason == ESP_SLEEP_WAKEUP_GPIO) {
        Serial.println("[WAKE] GPIO wakeup - checking hold...");
        checkWakeHold();  // Returns only if WAKE_HOLD_MS confirmed; else sleeps
        Serial.println("[WAKE] Hold confirmed - resuming");
        ledBlink(2, 60, 80);
    } else {
        Serial.println("[BOOT] Cold start");
    }

    // Restore settings from NVS
    prefs.begin(NVS_NS, false);
    sensIdx     = constrain(prefs.getUChar(NVS_SENS, 1), 0, 2);
    currentMode = (prefs.getUChar(NVS_MODE, 0) == 1) ? Mode::MOUSE : Mode::GAMEPAD;
    Serial.printf("[NVS] Sensitivity: %d  Mode: %s\n",
                  sensIdx, currentMode == Mode::GAMEPAD ? "Gamepad" : "Mouse");

    // Init Nunchuk
    delay(50);  // Allow Nunchuk VCC rail to stabilise after P-MOS turns on
    Serial.printf("[NUNCHUK] Init SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);
    if (!nunchuk.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("[NUNCHUK] Init failed - check wiring");
        while (true) { ledOn(); delay(150); ledOff(); delay(150); }
    }
    nunchuk.filterAlpha = 0.15f;
    nunchuk.deadzone    = 12;
    Serial.println("[NUNCHUK] Ready");

    // Init BLE
    gamepad.begin("Nunchuk BLE");

    // Initial battery reading
    delay(200);
    uint8_t bat = readBatteryPercent();
    gamepad.updateBattery(bat);
    Serial.printf("[BAT] Initial: %d%%\n", bat);

    // Mode indicator: 1 blink = Gamepad, 2 blinks = Mouse
    delay(200);
    ledBlink(currentMode == Mode::GAMEPAD ? 1 : 2);

    uint32_t now      = millis();
    st.lastUpdateMs   = now;
    st.lastMotionMs   = now;
    st.lastBatMs      = now;
    st.lastLedMs      = now;
    st.lastSwipeMs    = now;
    // st.stillStartMs = now;  // Uncomment when enabling still-lock

    Serial.println("============================================");
    Serial.printf("[MODE] %s\n", currentMode == Mode::GAMEPAD ? "Gamepad" : "Mouse");
    Serial.printf("[SENS] %s\n", sensIdx == 0 ? "Low" : sensIdx == 1 ? "Mid" : "High");
    Serial.println("[CTRL] Hold C+Z 2s    = toggle mode");
    Serial.println("[CTRL] Hold C 1.5s    = cycle sensitivity (Mouse only)");
    Serial.println("[CTRL] Hold power 2s  = sleep / wake");
    Serial.println("[CTRL] Swipe right    = Page Down / next slide (Mouse only)");
    Serial.println("[CTRL] Swipe left     = Page Up  / prev slide (Mouse only)");
    Serial.println("============================================\n");
}

// ════════════════════════════════════════════════════════════
//  Loop
// ════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // Power button runs every iteration (not throttled)
    checkPowerButton(now);

    // Heartbeat LED
    updateHeartbeat(now);

    // Auto-sleep: idle 10 min regardless of BLE connection state
    if ((now - st.lastMotionMs) >= AUTO_SLEEP_MS)
        goToSleep("Idle timeout (10 min)");

    // Battery level update
    updateBattery(now);

    // Rate throttle
    uint32_t interval = st.idleMode ? IDLE_MS : UPDATE_MS;
    if ((now - st.lastUpdateMs) < interval) return;
    st.lastUpdateMs = now;

    // Read Nunchuk
    NunchukData d;
    if (!nunchuk.read(d)) {
        Serial.println("[NUNCHUK] Read failed");
        return;
    }

    // Reserved: flip-pause (uncomment to enable)
    // if (checkFlipped(d)) { cacheInput(d); return; }

    // Sensitivity switch (Mouse mode, hold C alone)
    checkSensSwitch(d, now);

    // Mode switch (hold C+Z)
    if (checkModeSwitch(d, now)) {
        currentMode = (currentMode == Mode::GAMEPAD) ? Mode::MOUSE : Mode::GAMEPAD;
        prefs.putUChar(NVS_MODE, currentMode == Mode::MOUSE ? 1 : 0);
        // Reset swipe state on mode change
        st.prevGX      = d.gX;
        st.swipeFrames = 0;
        st.swipeDir    = 0;
        Serial.printf("[MODE] -> %s\n",
                      currentMode == Mode::GAMEPAD ? "Gamepad" : "Mouse");
        ledBlink(currentMode == Mode::GAMEPAD ? 1 : 2);
        cacheInput(d);
        return;
    }

    // Update idle tracking (any joystick or button activity resets timer)
    if (hasInputChanged(d)) {
        st.lastMotionMs = now;  // Reset auto-sleep timer (joystick/buttons only)
    }
    if (hasMotionForThrottle(d)) {
        st.idleMode     = false;
    } else if ((now - st.lastMotionMs) > STILL_TIMEOUT) {
        if (!st.idleMode) {
            st.idleMode = true;
            Serial.println("[IDLE] Throttling to 20Hz");
        }
    }

    // Send HID report
    if (gamepad.connected) {

        if (currentMode == Mode::GAMEPAD) {
            // ── Gamepad report ───────────────────────────────
            gamepad.sendGamepadReport(
                d.joyX, d.joyY,
                d.accelX, d.accelY, d.accelZ,
                d.btnC, d.btnZ
            );

        } else {
            // ── Mouse report ─────────────────────────────────

            // Joystick -> cursor delta
            const SensConfig& s = SENS_TABLE[sensIdx];
            int8_t deltaX = joyToDelta(d.joyX, s);
            int8_t deltaY = joyToDelta(d.joyY, s);

            // Reserved: accelerometer cursor (uncomment to switch from joystick)
            // deltaX = angleToDelta(d.roll,  s.scale, s.maxDelta);
            // deltaY = angleToDelta(d.pitch, s.scale, s.maxDelta);

            // Reserved: still-lock (uncomment to enable)
            // if (checkCursorLock(d, now)) { deltaX = 0; deltaY = 0; }

            // Suppress buttons during hold gestures to prevent mis-clicks
            bool sendButtons = !st.bothWasDown && !st.sensTriggered;

            gamepad.sendMouseReport(
                deltaX, deltaY,
                0, 0,                   // Scroll wheel fixed at 0
                sendButtons && d.btnC,  // Left click
                sendButtons && d.btnZ,  // Right click
                false                   // Middle: replace with detectFlick(d) to enable
            );

            // ── Swipe gesture -> page navigation ─────────────
            // Runs after mouse report so cursor and gesture are independent
            int swipe = checkSwipe(d, now);
            if (swipe == +1) {
                Serial.println("[SWIPE] Right -> Page Down (next slide)");
                gamepad.sendKey(KEY_PAGE_DOWN);
            } else if (swipe == -1) {
                Serial.println("[SWIPE] Left -> Page Up (prev slide)");
                gamepad.sendKey(KEY_PAGE_UP);
            }
        }
    }

    cacheInput(d);

#if DEBUG_SERIAL
    static uint32_t lastPrintMs = 0;
    if ((now - lastPrintMs) >= 250) {
        lastPrintMs = now;
        const char* modeStr = currentMode == Mode::GAMEPAD ? "PAD  " : "MOUSE";
        const char* sensStr = sensIdx == 0 ? "Low" : sensIdx == 1 ? "Mid" : "High";
        Serial.printf("[%s|%s] Joy(%4d,%4d) gX=%5.2f dGX=%5.2f "
                      "P=%5.1f R=%5.1f C=%d Z=%d BLE=%d\n",
                      modeStr, sensStr, d.joyX, d.joyY,
                      d.gX, d.gX - st.prevGX,
                      d.pitch, d.roll,
                      d.btnC, d.btnZ, gamepad.connected);
    }
#endif
}