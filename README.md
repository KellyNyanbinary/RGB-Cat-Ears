The **RGB Cat Ears** is a 3D-printed, motion-reactive cat ear headband. Each ear has a strip of **WS2812B LEDs**. The LEDs are controlled by a **Seeed Studio XIAO ESP32C3**, which uses an **Adafruit MPU-6050** as a sensor. The headband is powered via a USB-C cable. This repository contains the Arduino code for the ESP32C3.

## Hardware

The full CAD model and design are available on [Onshape](https://cad.onshape.com/documents/aee8694863dfed44766b44b6/w/e9796f69ad3ce933161dcc72/e/1ec2e349ca86cca24512c082?configuration=List_xsrrYu7Js48prM%3DDefault&renderMode=0&uiState=68ea332e07faaa2b8cdbc222).

**Main Components**:
- Seed Studio XIAO ESP32-C3
- Adafruit MPU-6050 (I²C accelerometer + gyroscope)  
- WS2812B (22 LEDs per ear)  
- USB-C power input  
- 3D-printed TPU headband and PETG ears

## Development

This is just the workflow I personally use; feel free to adapt it.

1. Clone the repository.
2. Open the Arduino IDE.
3. Install the following libraries:
    1. Adafruit BusIO
    2. Adafruit GFX Library
    3. Adafruit MPU6050
    4. Adafruit NeoPixel
    5. Adafruit SSD1306
    6. Adafruit Unified Sensor
4. Select the XIAO_ESP32C3 as the board in Arduino IDE.