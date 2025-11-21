Below is a **highly professional, publication-ready, GitHub-friendly README.md**, written in a clean, technical, research-style format suitable for academic submission, portfolio use, or robotics project showcase.

It maintains a **formal tone**, clear structure, precision, and concise wording while preserving all technical details.

---

# **Autonomous Mobile Robot Navigation using D* Lite Path Planning and Vision-Based Localization**

<img height="350" alt="image" src="https://github.com/user-attachments/assets/7bf7845b-144a-4769-ae4b-dcd04044f082" />

[Robot Testing](https://github.com/user-attachments/assets/23aa6c1a-f91c-45e3-ab09-f521fd5c8e38)

This repository contains a complete implementation of an **autonomous mobile robot navigation system** that integrates:

* **D* Lite global path planning** (running on Raspberry Pi)
* **Vision-based obstacle detection and map generation**
* **AprilTag-based robot localization and orientation estimation**
* **Encoder- and IMU-based closed-loop motion control** (running on ESP32)
* **UDP communication** between high-level planning and low-level control

The system enables robust, dynamic, real-time navigation where the robot continuously senses the environment, updates its global map, replans when obstacles emerge, and executes precise motor commands for movement and turning.

---

## **1. System Overview**

The navigation system is divided into two cooperating subsystems:

### **Raspberry Pi – High-Level Planning & Perception**

* Captures live camera feed (Picamera2)
* Performs AprilTag detection for pose estimation
* Generates an occupancy grid via computer vision
* Executes the **D* Lite algorithm** for dynamic path planning
* Determines next motion primitive (turn/forward)
* Transmits commands to ESP32 over UDP

### **ESP32 – Low-Level Control**

* Receives commands from Raspberry Pi
* Controls motors through PWM
* Reads wheel encoders for distance tracking
* Integrates gyroscope data for yaw estimation
* Performs automatic motor calibration
* Executes movements with soft PWM transitions and stability checks

---

## **2. High-Level Architecture**

```
             ┌─────────────────────────────────────────┐
             │               Raspberry Pi              │
             │─────────────────────────────────────────│
             │ • Camera capture (1280×720)             │
             │ • AprilTag pose estimation              │
             │ • Occupancy grid construction           │
             │ • Dynamic map updates                   │
             │ • D* Lite shortest path computation     │
             │ • Command synthesis (turn/forward)      │
             │ • UDP transmission to ESP32             │
             └──────────────────────┬──────────────────┘
                                    │ UDP Commands
                                    ▼
             ┌─────────────────────────────────────────┐
             │                   ESP32                 │
             │─────────────────────────────────────────│
             │ • Command parser                        │
             │ • Encoder-based distance control        │
             │ • MPU6050 gyro-based turn control       │
             │ • PWM motor driver interface            │
             │ • Motor auto-calibration                │
             │ • Status reporting (ACK)                │
             └─────────────────────────────────────────┘
```

---

## **3. Key Features**

### **3.1 D* Lite Dynamic Path Planning**

* Full implementation of D* Lite (g, rhs, priority queue, km update)
* Supports dynamic replanning when obstacles appear/disappear
* Real-time path recomputation synchronized with camera frames
* Efficient memory footprint suitable for embedded computation

### **3.2 Computer Vision–Based Mapping**

* Frame is divided into fixed-sized cells (default: 50×50 pixels)
* Each cell classified as **free** or **occupied** using:

  * Local intensity standard deviation
  * Local brightness thresholds
* Robot’s own cells auto-cleared to prevent self-obstacles

### **3.3 AprilTag-Based Localization**

* Uses `pupil_apriltags` for robust tag detection
* Estimates:

  * Robot grid cell
  * Pixel center coordinates
  * Yaw angle from rotation matrix
  * Cardinal direction (N/E/S/W)

### **3.4 Motion Control (ESP32)**

* Accurate linear motion via wheel encoders (ticks → cm)
* Smooth PWM transitions using exponential smoothing
* Precise turning via:

  * Gyroscope integration
  * Drift-corrected yaw tracking
* Automatic motor offset calibration to correct wheel imbalance

### **3.5 UDP-Based Communication**

* Commands are sent as simple strings:

  ```
  FORWARD:<ticks>
  BACK:<ticks>
  LEFT
  RIGHT
  STOP
  ```
* ESP32 returns `"OK"` after command processing
* Ensures low-latency, lightweight communication

---

## **4. System Workflow**

1. User selects **START** and **GOAL** cells on GUI.
2. Raspberry Pi generates an initial grid and computes D* Lite path.
3. Robot location and orientation are obtained from AprilTag.
4. Pi generates next command (turn + move forward).
5. ESP32 executes movement using encoders and gyro feedback.
6. Camera updates grid periodically to detect new obstacles.
7. D* Lite replans dynamically if path is obstructed.
8. System repeats until goal cell is reached.

---

## **5. Hardware Setup**

### Required Components

* **ESP32 DevKit**
* **Raspberry Pi 3/4** with Picamera2
* **L298N or similar motor driver**
* **DC motors with encoders**
* **MPU6050 gyroscope/accelerometer**
* **AprilTag marker (Tag36h11)** on robot
* Robot chassis with wheels and battery supply

### Wiring Overview (ESP32)

* I2C for MPU6050:

  * SDA → GPIO 25
  * SCL → GPIO 26
* Motor driver pins:

  * IN1/IN2/IN3/IN4
  * ENA/ENB (via `ledcAttach`)
* Encoder inputs:

  * Left encoder → GPIO 27
  * Right encoder → GPIO 14

---

## **6. Software Dependencies**

### Raspberry Pi

```
opencv-python
numpy
picamera2
pupil-apriltags
Pillow
```

### ESP32 (Arduino)

* Wire.h
* WiFi.h
* WiFiUDP.h
* Built-in LEDC PWM functions

---

## **7. Running the System**

### **1. Start the ESP32**

* Flash the `.ino` file
* Connect to Wi-Fi
* ESP32 displays its local IP

### **2. Start the Raspberry Pi Program**

```bash
python3 dstar_main.py
```

### **3. In the GUI**

* Click a **START** cell
* Click a **GOAL** cell
* The robot begins autonomous navigation

### **4. Controls**

* **R** — Reset planner
* **ESC** — Quit program

---

## **8. Visualization**

The system produces:

* Real-time overlay window (grid + obstacles + path + robot orientation)
* Saved image at each iteration (`dstar_live_overlay.jpg`)
* Logged obstacle map images (`dstar_photos/`)

---

## **9. Future Enhancements**

This platform can be extended with:

* Robot localization with Reinforcement Learning
* Multi-robot coordination
* Path smoothing (splines, Reeds-Shepp)
* Kalman/Extended Kalman filtering for pose fusion

---

## **10. Acknowledgments & Team Contributions**

| Team Member             | Primary Contribution Area                                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------- |
| **Nevil Vataliya**      | Hardware/Software Integration; ESP32 firmware development; UDP communication protocol.      |
| **Prankit Vishwakarma** | Software development; complete implementation of the D* Lite algorithm; obstacle detection logic & grid processing. |
| **Mohammad Shahil**     | Hardware assembly; robot chassis construction; low-level ESP32 motor driver & MPU6050 sensor handling.              |
| **Parth Modi**          | Raspberry Pi configuration; high-level Python development for planning & vision pipeline; overall software support. |

This project would not be possible without the combined efforts, collaboration, and domain knowledge contributed by each team member.
