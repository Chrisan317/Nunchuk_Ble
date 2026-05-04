# Nunchuk BLE HID

Convert a Nintendo Wii Nunchuk into a wireless Bluetooth HID device using the **XIAO ESP32-C3** and **PlatformIO**. The device presents itself as a composite HID — simultaneously a Gamepad, Mouse, and Keyboard — and includes battery reporting, deep sleep power management, and gesture recognition.

---

## Features

- **Dual mode** — Gamepad and Mouse, switchable at runtime
- **Gamepad mode** — joystick X/Y axes + 3-axis accelerometer (Rx/Ry/Rz) + C/Z buttons
- **Mouse mode** — joystick controls cursor; C = left click, Z = right click
- **Swipe gesture** — left/right wrist flick sends Page Up / Page Down (ideal for PowerPoint)
- **Three sensitivity presets** — Low / Mid / High, persisted across reboots via NVS
- **Battery level reporting** — BLE Battery Service (UUID 0x180F), updated every 60 s
- **Deep sleep** — hold power button 2 s; wake requires a 2 s hold to prevent false wakes
- **Auto sleep** — triggers after 10 minutes of joystick/button inactivity
- **Heartbeat LED** — single blink every 2 s confirms the device is running

---

## Hardware

| Component | Details |
|---|---|
| MCU | Seeed XIAO ESP32-C3 |
| Controller | Nintendo Wii Nunchuk |
| Battery | LiPo (any size; 300–500 mAh recommended) |
| Power button | Momentary tactile switch |

### Wiring

| Nunchuk Pin | XIAO ESP32-C3 |
|---|---|
| VCC (red) | 3.3V |
| GND (white) | GND |
| SDA (green) | GPIO 6 |
| SCL (yellow) | GPIO 7 |

| Component | XIAO ESP32-C3 |
|---|---|
| LED (active HIGH) | GPIO 10 (onboard) |
| Power button | GPIO 3 → GND (internal pull-up) |
| Battery ADC | GPIO 2 / A0 (via 2:1 voltage divider) |

#### Battery voltage divider

The XIAO ESP32-C3 ADC maximum input is 3.3 V. A LiPo cell reaches 4.2 V fully charged, so a 2:1 resistor divider is required before GPIO 2.

```
BAT+ ── R1 (100 kΩ) ── GPIO 2 ── R2 (100 kΩ) ── GND
```

---

## Software

### Dependencies

| Library | Version |
|---|---|
| [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) | ^1.4.2 |

### Project structure

```
nunchuk-ble-hid/
├── platformio.ini
└── src/
    ├── main.cpp       # Application logic
    ├── nunchuk.h      # I2C driver + accelerometer processing
    └── ble_hid.h      # BLE HID composite device + Battery Service
```

### BLE HID Reports

| Report ID | Type | Payload |
|---|---|---|
| 1 | Gamepad | joyX, joyY (int8) · accelX/Y/Z (int16 LE) · buttons |
| 2 | Mouse | buttons · deltaX · deltaY · wheel · hwheel (all int8) |
| 3 | Keyboard | modifier (uint8) · keycode (uint8) |

---

## Controls

### Both modes

| Input | Action |
|---|---|
| Hold **C + Z** for 2 s | Toggle Gamepad ↔ Mouse mode |
| Hold **power button** for 2 s | Enter deep sleep |
| Hold **power button** for 2 s (sleeping) | Wake up |

### Gamepad mode

| Input | HID output |
|---|---|
| Joystick X/Y | Axes X/Y |
| Accelerometer | Axes Rx/Ry/Rz |
| C button | Button 1 |
| Z button | Button 2 |

### Mouse mode

| Input | Action |
|---|---|
| Joystick X/Y | Cursor movement |
| C button | Left click |
| Z button | Right click |
| Hold **C alone** for 1.5 s | Cycle sensitivity Low → Mid → High |
| Swipe right (wrist flick) | Page Down (next slide) |
| Swipe left (wrist flick) | Page Up (previous slide) |

---

## Configuration

All tunable parameters are defined as `constexpr` at the top of `main.cpp`.

### Timing

| Constant | Default | Description |
|---|---|---|
| `AUTO_SLEEP_MS` | 600 000 | Idle auto-sleep timeout (10 min) |
| `WAKE_HOLD_MS` | 2 000 | Hold duration required to confirm wake |
| `SLEEP_HOLD_MS` | 2 000 | Hold duration to trigger sleep |
| `SWITCH_HOLD_MS` | 2 000 | Hold C+Z to switch mode |
| `SENS_HOLD_MS` | 1 500 | Hold C to cycle sensitivity |
| `BAT_UPDATE_MS` | 60 000 | Battery report interval |

### Mouse sensitivity

| Index | Scale | Max delta | Use case |
|---|---|---|---|
| 0 — Low | 0.10 | 12 px | Precise / small screen |
| 1 — Mid | 0.18 | 20 px | General use (default) |
| 2 — High | 0.28 | 30 px | Fast / large display |

### Swipe gesture

| Constant | Default | Description |
|---|---|---|
| `SWIPE_DELTA_THRESH` | 0.18 g | Minimum per-frame gX change to count |
| `SWIPE_CONFIRM_FRAMES` | 3 | Consecutive frames required to confirm |
| `SWIPE_COOLDOWN_MS` | 800 | Minimum ms between swipe triggers |

Swipe detection uses the **per-frame delta of filtered gX** (not absolute value), so gravity has no effect on triggering. Increase `SWIPE_DELTA_THRESH` if accidental triggers occur; decrease it if intentional swipes are missed.

---

## Reserved Accelerometer Features

The following features are fully implemented in `main.cpp` but commented out. Uncomment the relevant lines to enable them.

| Feature | How to enable | Effect |
|---|---|---|
| Flick → middle click | Uncomment `detectFlick(d)` call | Quick wrist flick sends middle mouse button |
| Still lock | Uncomment `checkCursorLock(d, now)` call | Freezes cursor when controller is at rest |
| Flip pause | Uncomment `checkFlipped(d)` call | Pauses all input when Nunchuk is upside-down |

---

## LED Behaviour

| Pattern | Meaning |
|---|---|
| Single blink every 2 s | Normal operation |
| 1 blink on boot | Started in Gamepad mode |
| 2 blinks on boot | Started in Mouse mode |
| 2 fast blinks | Woke from deep sleep |
| Solid ON → 2 blinks → fade | Entering deep sleep |
| Rapid flashing (error) | Nunchuk I2C init failed |

---

## NVS Persistence

The following settings survive power cycles and deep sleep:

| Key | Description |
|---|---|
| `sens` | Sensitivity index (0 / 1 / 2) |
| `mode` | Last active mode (0 = Gamepad, 1 = Mouse) |

NVS namespace: `nunchuk`

---

## Notes

### Re-pairing after firmware changes

If you modify the HID Report Descriptor in `ble_hid.h` (add/remove report IDs, change payload size), the host OS caches the old descriptor and will not update it automatically. You must **remove the device from the host's Bluetooth settings and pair again**.

Changes to `main.cpp` logic only do not require re-pairing.

### ESP32-C3 deep sleep GPIO wakeup

The XIAO ESP32-C3 maintains GPIO pull-ups during deep sleep, so no external pull-up resistor is needed on the power button pin. The firmware uses `esp_deep_sleep_enable_gpio_wakeup()` with `ESP_GPIO_WAKEUP_GPIO_LOW` on GPIO 3 (an RTC-capable pin).

### ADC accuracy

Battery percentage uses `analogReadMilliVolts()` instead of raw `analogRead()`. This applies Espressif's built-in ADC nonlinearity correction, giving more accurate voltage readings across the LiPo discharge curve (3.0 V – 4.2 V).

---

## License

MIT
