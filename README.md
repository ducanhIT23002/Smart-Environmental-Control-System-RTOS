# 🌱 Smart Environmental Control System (SECS) using RTOS

![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-20232A?style=for-the-badge&logo=freertos&logoColor=green)
![ARM](https://img.shields.io/badge/ARM_Cortex--M3-0091BD?style=for-the-badge&logo=arm&logoColor=white)

## 📌 Project Overview
The Smart Environmental Control System is a multi-threaded embedded system designed to monitor environmental conditions and autonomously control actuators. Developed on the **NXP LPC1768 (ARM Cortex-M3)** microcontroller, the project demonstrates advanced real-time operating system concepts by migrating from a basic CMSIS-RTOS wrapper to a fully native **FreeRTOS** architecture.

The system serves as the core for smart applications such as smart greenhouses or automated plant care systems, ensuring thread-safe operations, low-power states, and intelligent decision-making.

## ✨ Key Features & Technical Highlights

### 1. Tri-Mode Isolated Architecture (NEW)
The system operates under three strictly isolated modes to prevent logic collision and race conditions:
* **AUTO Mode:** Fully autonomous control driven by real-time sensor data and intelligent thresholds.
* **TIMER Mode:** Schedule-based execution handled by a background RTC daemon, fully decoupled from the UI.
* **MANUAL Mode:** Direct hardware intervention via physical UI joystick or remote UART commands.

### 2. Native FreeRTOS Implementation & Heap Optimization
* **Task Management:** Re-architected system core to support **10 concurrent threads** (Sensor, UART TX/RX, Menu, 3x Monitor, 3x Control). Optimized `configMINIMAL_STACK_SIZE` to prevent FreeRTOS heap exhaustion.
* **Producer-Consumer Pattern:** Implemented `monitor_thread` (Producer) and `control_thread` (Consumer) communicating via `xSemaphoreCreateBinary()`. This decoupled design ensures the system never blocks during hardware delays.
* **Resource Synchronization:** Used `xSemaphoreCreateMutex()` to prevent screen tearing on the GLCD and ensure thread-safe access to shared ADC hardware.

### 3. Intelligent Multi-Condition Control
Moved beyond simple threshold toggles by implementing context-aware actuator logic.
* **Smart Irrigation:** The sprinkler system evaluates both soil moisture and sunlight intensity. It prevents watering during harsh sunlight (Light > 3000) to protect plant roots from boiling effects, logging a warning via UART instead.

### 4. Advanced Graphical User Interface (GLCD)
* Built an event-driven, hierarchical menu system navigated via a 5-way joystick with robust hardware debouncing.
* **Dynamic Rendering:** Implemented calculation-based UI offset rendering (using `strlen`) and selective frame updates to eliminate screen flickering.
* **Energy Saving Mode:** A dynamic screen timeout mechanism drops the UI polling rate and powers down the GLCD after 10 seconds of inactivity, redirecting CPU cycles to critical core tasks.

### 5. Asynchronous UART Communication
* Real-time telemetry data logging to PC via UART.
* Non-blocking `uart_receive_thread` capable of parsing string commands (e.g., `CMD:Sprinkler:ON\n`) to remotely control the system when in MANUAL mode.

## 🛠️ Hardware Stack
* **Microcontroller:** NXP LPC1768 (ARM Cortex-M3)
* **Display:** Graphical LCD (GLCD) via SPI
* **Sensors (ADC):** Soil Moisture, Temperature/Heater state, Light intensity
* **Actuators (GPIO):** Water Pump (Sprinkler), Heater, Lighting
* **Input:** 5-way Joystick

## 🧠 System Architecture
The system operates on an event-driven and time-triggered hybrid architecture:
1. **`sensor_thread`**: Periodically acquires ADC data with mutex protection.
2. **`monitor_thread` (Daemon)**: Acts as a background watchdog, continuously tracking sensor thresholds (in AUTO mode) and RTC clocks (in TIMER mode), issuing semaphores when conditions are met.
3. **`control_thread`**: Consumes semaphores to execute intelligent logic and toggle GPIO actuators without blocking the main system.
4. **`menu_thread`**: Manages GLCD state, user input, and the Energy Saving state machine.

## 👨‍💻 Author
**Dương Hoàng Đức Anh**
*Undergraduate Student at International University - VNU HCMC*
* Driven by a passion for creating structured, logical, and user-centric systems, blending deep technical embedded execution with a strong product mindset.