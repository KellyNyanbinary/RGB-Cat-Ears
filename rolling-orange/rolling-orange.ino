#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define LED_PIN_0 D0
#define LED_PIN_1 D1
#define LED_COUNT_0 22
#define LED_COUNT_1 22
#define GAP_SIZE 3

Adafruit_NeoPixel strip0(LED_COUNT_0, LED_PIN_0, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip1(LED_COUNT_1, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);

  Wire.begin(6, 7);  // SDA, SCL for XIAO ESP32-C3
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  strip0.begin();
  strip0.setBrightness(128);
  strip0.show();

  strip1.begin();
  strip1.setBrightness(128);
  strip1.show();
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Gyro in deg/s
  float rollRate  = g.gyro.x * 57.2958;
  float pitchRate = g.gyro.y * 57.2958;
  float yawRate   = g.gyro.z * 57.2958;

  // Maximum rate we care about (clamp extremes)
  const float maxRate = 100.0;

  // Map signed rates to 0–255 with 128 as neutral gray center
  auto signedMap = [&](float rate) -> uint8_t {
    rate = constrain(rate, -maxRate, maxRate);
    return (uint8_t)map(rate, -maxRate, maxRate, 0, 255);
  };

  uint8_t red   = signedMap(rollRate);
  uint8_t green = signedMap(pitchRate);
  uint8_t blue  = signedMap(yawRate);

  // Optional: desaturate around center for smoother neutral zone
  const float deadband = 15.0;
  if (fabs(rollRate)  < deadband) red   = 128;
  if (fabs(pitchRate) < deadband) green = 128;
  if (fabs(yawRate)   < deadband) blue  = 128;

  stripCycle(60, (uint8_t)red, (uint8_t)green, (uint8_t)blue);

  // Optional: smooth the response
  // static float rAvg = 0, gAvg = 0, bAvg = 0;
  // float alpha = 0.2;  // smoothing factor (0–1)
  // rAvg = (1 - alpha) * rAvg + alpha * red;
  // gAvg = (1 - alpha) * gAvg + alpha * green;
  // bAvg = (1 - alpha) * bAvg + alpha * blue;

  // stripCycle(60, (uint8_t)rAvg, (uint8_t)gAvg, (uint8_t)bAvg);
}

// Updated stripCycle uses dynamic RGB color
void stripCycle(uint8_t wait, uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < GAP_SIZE; i++) {
    for (uint16_t j = 0; j < strip0.numPixels(); j++) {
      if (j % GAP_SIZE == GAP_SIZE - i - 1) {
        strip0.setPixelColor(j, strip0.Color(r, g, b));
      } else {
        strip0.setPixelColor(j, 0);
      }
    }

    for (uint16_t j = 0; j < strip1.numPixels(); j++) {
      if (j % GAP_SIZE == i) {
        // Mirror color or complement
        strip1.setPixelColor(j, strip1.Color(b, g, r));
      } else {
        strip1.setPixelColor(j, 0);
      }
    }

    strip0.show();
    strip1.show();
    delay(wait);
  }
}