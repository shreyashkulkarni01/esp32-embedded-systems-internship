# ESP32 Embedded Systems Internship — BMS Engineering Program

A 6-task embedded systems internship at Elevance Skill Development, building a progressively more capable Battery Management System (BMS) on the ESP32 platform.

## Description

This repository documents a hands-on embedded systems internship focused on designing a complete Battery Management System pipeline from the ground up. Starting with a modular cell-monitoring engine, the project progressively adds safety and protection logic, a flicker-free display interface, structured fault handling, live telemetry, and finally an enterprise-grade analytics dashboard. It's built for battery-powered systems that need real-time monitoring, fault tolerance, and remote visibility — and serves as a demonstration of non-blocking, state-machine-driven embedded design on the ESP32.

## Features

- **Modular BMS engine** — scalable cell-array architecture (4 to 16 cells), tracks weakest/strongest cell and voltage imbalance, adaptive SoC-based thresholds
- **Non-blocking protection relay** — hysteresis/debounce anti-chatter, statistical sensor anomaly detection, timed fault recovery
- **Flicker-free LCD engine** — partial-update rendering, non-blocking page rotation, fault-screen override
- **Structured fault state machine** — NORMAL/DEGRADED/FAILSAFE/SHUTDOWN states, fault source isolation, timestamped transition logging
- **Live telemetry via Blynk** — event-driven transmission, offline queueing during outages, RSSI-monitored Wi-Fi reconnection
- **Enterprise analytics dashboard** — historical trends, composite risk scoring, fault history, operator recommendations, executive summary

## Tech Stack

- **Hardware/Platform:** ESP32
- **Language:** C++ (Arduino framework)
- **Connectivity/Dashboard:** Blynk IoT platform
- **Core techniques:** non-blocking state machines, event-driven design, hysteresis/debounce filtering, statistical anomaly detection

## Installation / Setup

1. Clone this repository or download the task folder you need.
2. Open the relevant `.ino` file in the Arduino IDE (or import into [Wokwi](https://wokwi.com) for simulation).
3. Install required libraries: ESP32 board package, Blynk library, and LCD library (for Task 3).
4. For Tasks 5 and 6, update your Wi-Fi credentials and Blynk auth token in the code before uploading.

## Usage / How to Run

1. Select your ESP32 board and correct COM port in the Arduino IDE.
2. Upload the sketch to your ESP32 (or run directly in Wokwi if simulating).
3. Open the Serial Monitor to view logs, state transitions, and fault events.
4. For Tasks 5 and 6, open the Blynk app/dashboard to view live telemetry, trends, and risk scores in real time.
