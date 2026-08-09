## This is the readme for the Electrical folder. In here will be the explanation for the circuit and any relevant variations; As well as reasoning behind our electronic choices .
---


## Schematic
<img src="/Electrical/WRO_FE_2026_schematic_schem.png" width="700" height="700">

---

## Subsystems Overview and Electronics & Wiring

### 1. Power Distribution
* **Main Power Source:** 3S LiPo Battery (11.1V, 2200mAh) acts as the source of main power for the system by means of a Master Rocker Switch.
* **Logic Power (5V Rail):** In-line 5V Voltage Regulator reduces the 11.1V battery voltage to supply power to the Raspberry Pi 5 via the USB-C PWR port.
* **Actuators Power:** Furitek Lizard Pro ESC is powered directly from the battery bus. Its built-in 6V/3A BEC regulates power to the actuators and peripheral logic.

---

### 2. Microcontroller & Compute Units

#### **Raspberry Pi 5 (High-Level Computer Vision Controller)**
* In charge of high-level computer vision, image processing, and path planning.
* **Camera:** Interfaced using the MIPI CSI port to a Raspberry Pi camera for lane detection and color code reading.
* **Serial Bridge:** Interfaced using a USB 2.0 connection to the low-level motor controller.

#### **Arduino Nano v2.3 (Low-Level Motor Controller)**
* In charge of real-time motor control and real-time PWM execution.
* **ESC Signal:** Sends PWM control signals through pin D4 to the ESC.
* **Servo Signal:** Controls the steering mechanism using PWM signals on pin D3.
* **Power:** Powered through the USB rail / Power bus interfacing to the main controller hub.

---

### 3. Actuators & Motors
* **Primary Motor:** Furitek Micro Komodo 1212 (3450KV) Brushless DC Motor connected through three phase leads (MOTOR A, B, C).
* **ESC:** Furitek Lizard Pro 2S-4S LiPo ESC (45A continuous / 180A burst rating) for Brushless DC motor commutation.
* **Steering Motor:** Standard 3-pin PWM Servo Motor interfaced to pin D3 of the Arduino Nano.

---



## Why we changed our robot





### 4. Sensors and Future Hardware Expansion
* **Vision:** Raspberry Pi camera for visual tracking in real time.
* **D500 LiDAR (Not Connected / Plugged in):**
  * **Status:** The D500 LiDAR sensor with JST ZH-5 and JST ZH-6 connectors for DATA1, SPEED CONTROL1, and POWER & GND1 is mounted on the chassis but not yet plugged in.
  * **Future Scope:** Will be connected to the Raspberry Pi’s UART or USB port for 2D/3D obstacle grid detection and wall-following algorithms.
