# FRSBot - 12-DOF ESP32 Spider Quadruped

FRSBot is a four-legged, spider-style robot powered by an ESP32 DevKit V1 and twelve MG90S servos. This version adds phone-based wireless control through the ESP32's built-in Bluetooth Classic SPP interface. No external HC-05/HC-06 Bluetooth module is required when using a Bluetooth Classic-capable original ESP32 board (for example, ESP32-WROOM-32).

## Features

- 12-DOF motion: three servos per leg (hip, lift, foot)
- Continuous forward and backward walking
- Left and right turning gait cycles
- Smooth quintic interpolation
- Android phone control over Bluetooth Classic SPP
- USB Serial control remains available for testing
- Stop/stand command and Bluetooth-disconnect stop handling

## Hardware

- ESP32 DevKit V1 with Bluetooth Classic support
- 12 x MG90S servo motors
- External regulated 5 V servo power supply sized for the servo load
- Common ground between the servo supply and ESP32

**Power note:** Do not power all twelve servos from the ESP32 3.3 V pin or its onboard regulator. Use a separate 5 V supply with adequate current capacity and connect grounds together.

## Servo mapping

| Servo | Leg | Function | ESP32 GPIO |
|---|---|---|---:|
| S1 | Back Left | Foot | 13 |
| S2 | Back Left | Lift | 14 |
| S3 | Back Left | Hip | 27 |
| S4 | Front Left | Foot | 26 |
| S5 | Front Left | Lift | 33 |
| S6 | Front Left | Hip | 32 |
| S7 | Back Right | Hip | 25 |
| S8 | Back Right | Foot | 19 |
| S9 | Back Right | Lift | 21 |
| S10 | Front Right | Hip | 18 |
| S11 | Front Right | Lift | 23 |
| S12 | Front Right | Foot | 22 |

## Bluetooth phone control

1. Upload `FRSBOT_Bluetooth_Phone_Remote.ino` to a Bluetooth Classic-capable ESP32 board.
2. Open Bluetooth settings on an Android phone and pair with **FRSBOT**.
3. Install an SPP-compatible app such as [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal).
4. Connect to **FRSBOT** in the app.
5. Send one character for each command:

| Command | Action |
|---|---|
| `F` | Continuous forward walking |
| `B` | Continuous backward walking |
| `L` | Continuous left turn |
| `R` | Continuous right turn |
| `X` | Stop and return to normal standing |
| `S` | Normal standing |

The same commands can be sent through the USB Serial Monitor at **115200 baud**.

## Software requirements

- Arduino IDE
- ESP32 board package
- `ESP32Servo` library
- ESP32 Arduino core with `BluetoothSerial` / Bluetooth Classic SPP support

This sketch uses Bluetooth Classic SPP. It is not suitable for ESP32 variants that support BLE only (such as ESP32-C3 and ESP32-S3) without changing the communication implementation.

## Safety and calibration

- Test the robot with its legs off the ground before a floor test.
- Check mechanical clearance at all calibrated angles, especially the turning hip targets.
- Keep hands, wires, and loose objects clear of linkages.
- Confirm the servo power supply can handle simultaneous servo current.
- Servo direction depends on horn mounting and mechanical geometry; validate each movement on the assembled robot.

## Repository contents

- `FRSBOT_Bluetooth_Phone_Remote.ino` - ESP32 firmware with Bluetooth phone control.
- `README.md` - Hardware, mapping, setup, and command documentation.

---

Project: FRSBot 12-DOF Spider Quadruped
Controller: ESP32 DevKit V1
Wireless control: Bluetooth Classic SPP
