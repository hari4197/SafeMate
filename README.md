# 🏥 SafeMate – Patient-Room Safety & Environment Monitor

SafeMate is a simple patient-room safety and environment monitor built using **ESP32-C3**.  
It tracks **temperature, humidity, motion, and vibration**, and alerts caregivers using **LED indicators, a buzzer, and a 16×2 I²C LCD**.  

---

## 🔧 Features

- Continuous **temperature & humidity monitoring** (DHT11)
- **Motion detection** via PIR sensor
- **Vibration / fall detection** via vibration switch
- **Multi-level visual alerts** using Green / Yellow / Red LEDs
- **Audible buzzer alarm** with **Silence** and **Reset** buttons
- **16×2 I²C LCD status display** (live sensor values + alarm status)
- Built with **ESP-IDF** and **FreeRTOS** for reliable multitasking

---

## 📦 Components Used

- ESP32-C3-DevKitM-1
- DHT11 Temperature & Humidity Sensor  
- PIR Motion Sensor  
- Vibration Sensor
- 16×2 LCD with I²C backpack (PCF8574, address `0x27`)  
- LEDs (Green, Yellow, Red) + current-limiting resistors  
- Active Buzzer  
- Push buttons (Silence, Reset)  

---

## 🏗️ System Overview

- **Inputs**  
  - DHT11 → Temperature & Humidity  
  - PIR → Motion detection  
  - Vibration sensor → Shock / fall events  
  - Buttons → User control (Silence / Reset)

- **Outputs**  
  - LEDs → Safety status indication  
  - Buzzer → Audible alarm  
  - LCD → Live readings and event status  

---

## 📂 Project Structure

```text
Project_SafeMate/
├─ main/
│  ├─ Project_SafeMate.c      # Main application logic
│  └─ CMakeLists.txt          # Component registration
├─ components/
│  └─ (optional custom components)
├─ managed_components/
│  └─ espressif__esp_rainmaker  # Added via idf.py add-dependency
├─ sdkconfig                  # ESP-IDF configuration
├─ sdkconfig.defaults         # Default config (board-independent)
└─ CMakeLists.txt             # Top-level CMake project file

