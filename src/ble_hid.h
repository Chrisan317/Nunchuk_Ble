#pragma once
// ============================================================
//  ble_hid.h - Composite BLE HID device + Battery Service
//
//  Report ID 1: Gamepad
//    [0]   joyX        int8   -127~127
//    [1]   joyY        int8   -127~127
//    [2-3] accelX      int16  LE  -512~511
//    [4-5] accelY      int16  LE  -512~511
//    [6-7] accelZ      int16  LE  -512~511
//    [8]   buttons     bit0=C  bit1=Z
//
//  Report ID 2: Mouse
//    [0]   buttons     bit0=Left(C)  bit1=Right(Z)  bit2=Middle(reserved)
//    [1]   deltaX      int8  cursor X (relative)
//    [2]   deltaY      int8  cursor Y (relative)
//    [3]   wheel       int8  vertical scroll   (fixed 0)
//    [4]   hwheel      int8  horizontal scroll (fixed 0)
//
//  Report ID 3: Keyboard (minimal - Page Up / Page Down only)
//    [0]   modifier    uint8  modifier keys bitmask (always 0 here)
//    [1]   keycode     uint8  HID keycode
//          0x4B = Page Up   (swipe right = next slide)
//          0x4E = Page Down (swipe left  = prev slide)
//          0x00 = key release
//
//  Battery Service: UUID 0x180F / Characteristic 0x2A19
// ============================================================
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>

static const uint8_t HID_REPORT_DESC[] = {

    // ── Gamepad (Report ID 1) ────────────────────────────────
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01,
    // Joystick X/Y (int8 each)
    0x09, 0x30, 0x09, 0x31,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02,
    // Accelerometer Rx/Ry/Rz (int16 each, LE)
    0x09, 0x33, 0x09, 0x34, 0x09, 0x35,
    0x16, 0x00, 0xFE, 0x26, 0xFF, 0x01, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02,
    // Buttons C/Z + padding to byte boundary
    0x05, 0x09, 0x19, 0x01, 0x29, 0x02,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02,
    0x95, 0x06, 0x75, 0x01, 0x81, 0x03,
    0xC0,

    // ── Mouse (Report ID 2) ──────────────────────────────────
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x09, 0x01, 0xA1, 0x00, 0x85, 0x02,
    // 3 buttons (Left / Right / Middle) + 5-bit padding
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03, 0x81, 0x02,
    0x95, 0x05, 0x75, 0x01, 0x81, 0x03,
    // X/Y cursor (relative int8)
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
    // Vertical scroll wheel (relative int8)
    0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    // Horizontal scroll - Consumer Page (relative int8)
    0x05, 0x0C, 0x0A, 0x38, 0x02,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    0xC0, 0xC0,

    // ── Keyboard (Report ID 3) ───────────────────────────────
    // Minimal keyboard report: 1 modifier byte + 1 keycode byte
    // Used only for Page Up / Page Down (no modifier needed)
    // Keycode 0x4B = Page Up, 0x4E = Page Down, 0x00 = release
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    // Modifier byte (Ctrl/Shift/Alt etc.) - always 0 for page keys
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        //   Usage Minimum (Left Ctrl)
    0x29, 0xE7,        //   Usage Maximum (Right GUI)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1 bit)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    // Keycode byte
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data, Array, Absolute)
    0xC0               // End Collection
};

// ── Keyboard keycodes (HID Usage Table, Keyboard page) ───────
constexpr uint8_t KEY_PAGE_UP   = 0x4B;  // Page Up
constexpr uint8_t KEY_PAGE_DOWN = 0x4E;  // Page Down
constexpr uint8_t KEY_RELEASE   = 0x00;  // Key release

class BLEHIDGamepad : public NimBLEServerCallbacks {
public:
    bool connected    = false;
    bool paired       = false;
    bool shuttingDown = false;  // Set true before disconnect to suppress re-advertising

    void begin(const char* deviceName = "Nunchuk BLE") {
        NimBLEDevice::init(deviceName);
        NimBLEDevice::setSecurityAuth(true, false, true);
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

        _server = NimBLEDevice::createServer();
        _server->setCallbacks(this);

        _hid = new NimBLEHIDDevice(_server);
        _hid->manufacturer()->setValue("Nintendo DIY");
        _hid->pnp(0x02, 0x057E, 0x0306, 0x0110);
        _hid->hidInfo(0x00, 0x01);
        _hid->reportMap((uint8_t*)HID_REPORT_DESC, sizeof(HID_REPORT_DESC));
        _gamepadChar  = _hid->inputReport(1);
        _mouseChar    = _hid->inputReport(2);
        _keyboardChar = _hid->inputReport(3);
        _hid->startServices();

        // Battery Service (UUID 0x180F / Characteristic 0x2A19)
        NimBLEService* batSvc = _server->createService(NimBLEUUID((uint16_t)0x180F));
        _batChar = batSvc->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A19),
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        uint8_t initBat = 100;
        _batChar->setValue(&initBat, 1);
        batSvc->start();

        _startAdvertising(deviceName);
    }

    // Update battery level 0-100, notifies connected host
    void updateBattery(uint8_t pct) {
        if (!_batChar) return;
        pct = min(pct, (uint8_t)100);
        _batChar->setValue(&pct, 1);
        if (connected) _batChar->notify();
    }

    // Send Gamepad report (Report ID 1), 9 bytes
    void sendGamepadReport(int8_t joyX, int8_t joyY,
                           int16_t ax, int16_t ay, int16_t az,
                           bool btnC, bool btnZ) {
        if (!connected) return;
        ax = constrain(ax, -512, 511);
        ay = constrain(ay, -512, 511);
        az = constrain(az, -512, 511);
        uint8_t r[9];
        r[0] = (uint8_t)joyX;
        r[1] = (uint8_t)joyY;
        r[2] = ax & 0xFF;        r[3] = (ax >> 8) & 0xFF;
        r[4] = ay & 0xFF;        r[5] = (ay >> 8) & 0xFF;
        r[6] = az & 0xFF;        r[7] = (az >> 8) & 0xFF;
        r[8] = (btnC ? 0x01 : 0) | (btnZ ? 0x02 : 0);
        _gamepadChar->setValue(r, sizeof(r));
        _gamepadChar->notify();
    }

    // Send Mouse report (Report ID 2), 5 bytes
    // btnMid: middle click - reserved for flick gesture, default false
    void sendMouseReport(int8_t deltaX, int8_t deltaY,
                         int8_t wheel,  int8_t hwheel,
                         bool btnC, bool btnZ, bool btnMid = false) {
        if (!connected) return;
        uint8_t r[5];
        r[0] = (btnC   ? 0x01 : 0)
             | (btnZ   ? 0x02 : 0)
             | (btnMid ? 0x04 : 0);
        r[1] = (uint8_t)deltaX;
        r[2] = (uint8_t)deltaY;
        r[3] = (uint8_t)wheel;
        r[4] = (uint8_t)hwheel;
        _mouseChar->setValue(r, sizeof(r));
        _mouseChar->notify();
    }

    // Send keyboard key press then release (Report ID 3)
    // keycode: KEY_PAGE_UP or KEY_PAGE_DOWN
    void sendKey(uint8_t keycode) {
        if (!connected) return;
        uint8_t r[2];
        // Key down: modifier=0, keycode
        r[0] = 0x00;
        r[1] = keycode;
        _keyboardChar->setValue(r, sizeof(r));
        _keyboardChar->notify();
        delay(30);  // Hold long enough for host to register
        // Key release: all zeros
        r[0] = 0x00; r[1] = 0x00;
        _keyboardChar->setValue(r, sizeof(r));
        _keyboardChar->notify();
    }

    void onConnect(NimBLEServer* pServer) override {
        connected = true;
        paired    = true;
        Serial.println("[BLE] Connected");
        pServer->updateConnParams(
            pServer->getPeerInfo(0).getAddress(), 12, 24, 0, 400);
    }

    void onDisconnect(NimBLEServer*) override {
        connected = false;
        if (shuttingDown) {
            Serial.println("[BLE] Disconnected (shutting down)");
            return;
        }
        Serial.println("[BLE] Disconnected - restarting advertising");
        NimBLEDevice::getAdvertising()->start();
    }

private:
    NimBLEServer*         _server       = nullptr;
    NimBLEHIDDevice*      _hid          = nullptr;
    NimBLECharacteristic* _gamepadChar  = nullptr;
    NimBLECharacteristic* _mouseChar    = nullptr;
    NimBLECharacteristic* _keyboardChar = nullptr;
    NimBLECharacteristic* _batChar      = nullptr;

    void _startAdvertising(const char* name) {
        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        adv->setName(name);
        adv->setAppearance(0x03C2);
        adv->addServiceUUID(_hid->hidService()->getUUID());
        adv->addServiceUUID(NimBLEUUID((uint16_t)0x180F));
        adv->setScanResponse(true);
        adv->setMinPreferred(0x06);
        adv->setMaxPreferred(0x12);
        adv->start();
        Serial.printf("[BLE] Advertising as: %s\n", name);
    }
};