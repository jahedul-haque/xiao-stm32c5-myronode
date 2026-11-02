# MYRONODE 🪞⚡
### Distributed Kinematic Mimic System via micro-ROS 2, CAN FD & Seeed Studio XIAO STM32C5

---

## 📌 Overview

**MyroNode** is a high-speed, distributed kinematic mirroring and teleoperation system built for two **Seeed Studio XIAO STM32C5** boards connected over a long-distance **CAN FD** bus.

* **Node 2 (Handheld Leader & micro-ROS Gateway)**:
  * Connects to the computer running the **ROS 2 / micro-ROS Agent** over USB CDC-ACM.
  * Continuously samples its onboard **ST LSM6DS3TR-C 6-axis IMU** (via `i2c2` on PB3/PB4) at 50 Hz.
  * Calculates smooth Pitch & Roll tilt angles using an embedded complementary filter.
  * In **MIMIC MODE**, it maps tilt gestures directly to target joint angles ($0^\circ \text{ to } 300^\circ$) and broadcasts command frames over CAN FD (`ID 0x210`).
  * Receives actual servo telemetry over CAN FD (`ID 0x110`) and exposes standard ROS 2 topics (`/node2/imu`, `/node1/servo_state`).

* **Node 1 (Actuator Follower)**:
  * Connected to Node 2 via CAN FD (FDCAN2 on PB5/PB13).
  * Controls the **Feetech SCS0009 serial bus servo** (or XIAO bus servo driver) over USART1 (PA9/PA10) at 1,000,000 bps (1 Mbps).
  * Drives the physical servo to mirror Node 2's orientation in real time.
  * Queries real-time present position from the servo and broadcasts 20-byte telemetry frames over CAN FD back to Node 2.

* **Minimalist Dashboard**:
  * An ultra-clean, dark-mode web control interface (`http://localhost:8090`).
  * Displays a live **Attitude Horizon** (Roll/Pitch of Node 2) and **Servo Angle Radial Dial** (Node 1 actual vs target angle).
  * Switch between **🟢 LIVE MIMIC (IMU)**, **🔵 MANUAL SLIDER**, and **⚪ HOLD CENTER (150°)** with a single click.

---

## 📐 Hardware Architecture & Wiring

```
    [ Hand Tilt Gesture ]
             │
             ▼
   ┌──────────────────┐
   │   XIAO Node 2    │  (Leader & micro-ROS Gateway)
   │  (On-board IMU)  │  - USB CDC-ACM -> Computer (micro-ROS / Dashboard)
   └────────┬─────────┘  - I2C2 -> ST LSM6DS3TR-C
            │
            │ CAN FD Bus (500k Nominal / 2M Data Bitrate)
            │ (CAN_H to CAN_H, CAN_L to CAN_L, 120Ω Terminated)
            ▼
   ┌──────────────────┐
   │   XIAO Node 1    │  (Follower Controller)
   │ (STM32C5 MCU)    │  - FDCAN2 Subsystem
   └────────┬─────────┘  - USART1 (PA9 TX, PA10 RX) @ 1,000,000 bps
            │
    Half-Duplex UART
            │
            ▼
   [ SCS0009 Bus Servo ] (Physical Joint Mimicking Hand Motion)
```

### Wiring Table

| Connection | Node 1 (Follower) | Node 2 (Leader / Gateway) | Peripheral | Description |
| :--- | :--- | :--- | :--- | :--- |
| **CAN_H** | CAN_H Pad | CAN_H Pad | `FDCAN2` | CAN FD differential high line |
| **CAN_L** | CAN_L Pad | CAN_L Pad | `FDCAN2` | CAN FD differential low line |
| **GND** | GND | GND | Power | Common signal ground |
| **Servo TX/RX** | `PA9` (TX) / `PA10` (RX)| — | `USART1` | 1 Mbps bus servo communication |
| **Servo Power**| 5V / 6V + GND | — | Power | External servo power rail |
| **USB CDC** | — | USB Type-C | `USB_DRD_FS`| Connects to PC running Dashboard / ROS 2 |

---

## 📂 Project Directory Structure

```
xiao-stm32c5-kinemirror/
├── node1-servo-follower/       # Firmware for Node 1: Bus Servo Controller
│   ├── platformio.ini          # PlatformIO board definition
│   ├── src/main.c              # SCS0009 driver, CAN FD command listener & telemetry TX
│   └── zephyr/                 # Zephyr RTOS overlays & configuration (USART1 @ 1 Mbps)
├── node2-imu-gateway/          # Firmware for Node 2: IMU Leader & Gateway
│   ├── platformio.ini          # PlatformIO board definition
│   ├── src/main.c              # LSM6DS3TR-C IMU filter, tilt mapping, CAN FD TX/RX, USB CDC
│   └── zephyr/                 # Zephyr RTOS overlays & configuration (I2C2, FDCAN2, USB)
├── dashboard/                  # Minimalist Dashboard & ROS 2 Bridge
│   ├── dashboard.py            # Standalone minimalist web dashboard (http://localhost:8090)
│   ├── ros2_bridge.py          # Native ROS 2 / rclpy node for micro-ROS ecosystem
│   └── requirements.txt        # Python dependencies (pyserial)
└── README.md
```

---

## 📡 CAN FD Message Protocol

### 1. Servo Command Frame (`CAN_ID_SERVO_CMD = 0x210`)
*Transmitted from Node 2 to Node 1 at 50 Hz:*
* **`servo_id`** (`uint8_t`): Target servo address (default `1`).
* **`target_pos`** (`int16_t`): Target position in counts (`0 to 1023`, corresponding to $0.0^\circ \text{ to } 300.0^\circ$).
* **`move_time_ms`** (`uint16_t`): Interpolation duration in milliseconds (`0` for instant tracking).
* **`move_speed`** (`uint16_t`): Velocity limit (`0` for maximum speed).

### 2. Servo Telemetry Frame (`CAN_ID_SERVO_DATA = 0x110`)
*Transmitted from Node 1 to Node 2 at 25 Hz:*
* **`servo_id`** (`uint8_t`): Originating servo ID.
* **`status_flags`** (`uint8_t`): `0 = OK`, `1 = Timeout / Simulated`.
* **`current_pos`** (`int16_t`): Real-time encoder position (`0 to 1023`).
* **`current_angle`** (`float`): Angle in degrees ($0.0^\circ \text{ to } 300.0^\circ$).
* **`target_pos`** (`int16_t`): Current target setpoint.
* **`target_angle`** (`float`): Target angle in degrees.
* **`uptime_ms`** (`uint32_t`): Node 1 system uptime.

---

## 🤖 ROS 2 & micro-ROS Topics

When running `ros2_bridge.py` or the `micro-ros-agent`:

| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| **`/node2/tilt`** | `geometry_msgs/msg/Vector3Stamped` | Publish | X = Roll (°), Y = Pitch (°), Z = Gyro Z (°/s) |
| **`/node1/servo_state`** | `sensor_msgs/msg/JointState` | Publish | Position (rad), Effort (counts) of SCS0009 |
| **`/servo/cmd`** | `std_msgs/msg/Float32` | Subscribe | Manual angle override ($0.0^\circ \text{ to } 300.0^\circ$) |
| **`/mimic/mode`** | `std_msgs/msg/Int32` | Subscribe | `0` = Mimic IMU, `1` = Manual Slider, `2` = Hold Center |

---

## ⚡ How to Build & Flash

### 1. Build and Flash Node 1 (Servo Follower)
1. In PlatformIO or terminal:
   ```bash
   cd H:\Projects\platformio\xiao-stm32c5-kinemirror\node1-servo-follower
   pio run
   ```
2. Double-click the reset button on Node 1 to mount `XIAOC5BOOT`.
3. Upload firmware:
   ```bash
   pio run -t upload
   ```
   *(Or drag `.pio/build/seeed-xiao-stm32c5/firmware.uf2` directly to `XIAOC5BOOT`).*

### 2. Build and Flash Node 2 (IMU Gateway)
1. In PlatformIO or terminal:
   ```bash
   cd H:\Projects\platformio\xiao-stm32c5-kinemirror\node2-imu-gateway
   pio run
   ```
2. Double-click the reset button on Node 2 to mount `XIAOC5BOOT`.
3. Upload firmware:
   ```bash
   pio run -t upload
   ```

---

## 🖥️ Launching the Minimalist Dashboard

1. Connect Node 2 to your computer via USB.
2. Install Python dependency:
   ```bash
   pip install pyserial
   ```
3. Launch the dashboard:
   ```bash
   cd H:\Projects\platformio\xiao-stm32c5-kinemirror\dashboard
   python dashboard.py
   ```
4. Open your browser at **`http://localhost:8090`**:
   * Select your XIAO COM port from the top dropdown and click **Connect**.
   * Tilt Node 2 in your hand: watch the horizon line rotate and the SCS0009 servo mirror the tilt in real time!
   * Switch between **🟢 LIVE MIMIC**, **🔵 MANUAL SLIDER**, and **⚪ HOLD CENTER**.

### (Optional) Running Native ROS 2 Bridge:
```bash
python ros2_bridge.py --ros-args -p port:=COM3
```
You can now visualize in RViz or echo topics:
```bash
ros2 topic echo /node1/servo_state
ros2 topic pub /servo/cmd std_msgs/msg/Float32 "{data: 90.0}"
```
