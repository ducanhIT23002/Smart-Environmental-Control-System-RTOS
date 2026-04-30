# 🌱 Smart Environmental Control System (SECS) using RTOS

![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-20232A?style=for-the-badge&logo=freertos&logoColor=green)
![ARM](https://img.shields.io/badge/ARM_Cortex--M3-0091BD?style=for-the-badge&logo=arm&logoColor=white)

## 📌 Project Overview
The Smart Environmental Control System is a multi-threaded embedded system designed to monitor environmental conditions and autonomously control actuators[cite: 3]. Developed on the **NXP LPC1768 (ARM Cortex-M3)** microcontroller, the project demonstrates advanced real-time operating system concepts by migrating from a basic CMSIS-RTOS wrapper to a fully native **FreeRTOS** architecture[cite: 2].

The system serves as the core for smart applications such as smart greenhouses or automated plant care systems[cite: 3], ensuring thread-safe operations, low-power states, and intelligent decision-making.

## ✨ Key Features & Technical Highlights

### 1. Native FreeRTOS Implementation
* **Task Management:** Re-architected system core to utilize native FreeRTOS `xTaskCreate` with precise priority and stack size allocation for 6 independent threads (Sensor, Control, Monitor, UART TX/RX, Menu)[cite: 2].
* **Resource Synchronization:** Implemented `SemaphoreHandle_t` using `xSemaphoreCreateMutex()` and `xSemaphoreCreateBinary()` to prevent race conditions and ensure thread-safe access to shared hardware (ADC, SPI GLCD) and data arrays[cite: 2].
* **Nested Mutex Strategy:** Designed a robust locking mechanism in the UI thread to prevent screen tearing and data corruption during concurrent hardware updates, successfully avoiding deadlock scenarios.

### 2. Intelligent Multi-Condition Control[cite: 3]
Moved beyond simple threshold toggles by implementing context-aware actuator logic.
* **Smart Irrigation:** The sprinkler system evaluates both soil moisture and sunlight intensity. It prevents watering during harsh sunlight (Light > 3000) to protect plant roots from boiling effects, logging a warning via UART instead[cite: 2].

### 3. Energy Saving Mode (Deep Sleep UI)[cite: 3]
* Implemented a dynamic screen timeout mechanism using FreeRTOS `xTaskGetTickCount()`[cite: 2].
* **CPU Optimization:** If no joystick activity is detected for 10 seconds, the GLCD enters a black-screen sleep state. The UI polling rate drops significantly (yielding CPU via `vTaskDelay`), redirecting maximum processing power to critical sensor and control tasks[cite: 2].

### 4. Interactive Graphical User Interface (GLCD)
* Built an event-driven, hierarchical menu system navigated via a 5-way joystick[cite: 3].
* Features robust switch debouncing logic to ensure smooth user experience[cite: 2].

### 5. Asynchronous UART Communication
* Real-time telemetry data logging to PC via UART[cite: 3].
* Non-blocking `uart_receive_thread` capable of parsing string commands (e.g., `Heater:ON`) to manually override automated system states via binary semaphores[cite: 2].

## 🛠️ Hardware Stack
* **Microcontroller:** NXP LPC1768 (ARM Cortex-M3)
* **Display:** Graphical LCD (GLCD) via SPI
* **Sensors (ADC):** Soil Moisture, Temperature/Heater state, Light intensity[cite: 3]
* **Actuators (GPIO):** Water Pump (Sprinkler), Heater, Lighting[cite: 3]
* **Input:** 5-way Joystick[cite: 3]

## 🧠 System Architecture
The system operates on an event-driven and time-triggered hybrid architecture:
1. **`sensor_thread`**: Periodically acquires ADC data with mutex protection[cite: 2, 3].
2. **`control_thread`**: Acts on binary semaphores triggered by manual UART commands, timers, or the `monitor_thread`'s multi-condition logic[cite: 2, 3].
3. **`menu_thread`**: Manages GLCD state, user input, and the Energy Saving state machine[cite: 2, 3].

## 👨‍💻 Author
**Dương Hoàng Đức Anh**
*Undergraduate Student at International University - VNU HCMC*
* Driven by a passion for creating structured, logical, and user-centric systems, blending deep technical embedded execution with a strong product mindset.