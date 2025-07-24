Crop Secuirty Project

An ESP32-CAM–based system combining environmental sensing (DHT11, ultrasonic), obstacle detection, flame sensing, and Telegram alerts with live camera snapshots.

## Features
- **Temperature & Humidity** via DHT11
- **Ultrasonic Distance** sensing
- **Light Level** monitoring (4 photoresistors)
- **Motor Control** (e.g., ventilation)
- **Flame Detection** via digital sensor
- **OLED Display** for real-time status
- **Telegram Bot Integration**: send alerts and photos over Wi-Fi

## Getting Started
1. Clone repo  
2. Open `Crop_secuirty.inoo` in Arduino IDE  
3. Install libraries:  
   - LiquidCrystal  
   - DFRobot_DHT11  
   - WiFiClientSecure  
   - UniversalTelegramBot  
   - ArduinoJson  
   - NewPing  
   - ESP32 camera support  
4. Update `ssid`, `password`, `BOTtoken`, `chatId`  
5. Wire per comments in code  
6. Upload to ESP32–CAM  

Alerts trigger on flame, obstacle (<10 cm), or button press. Photos sent to Telegram chat.

