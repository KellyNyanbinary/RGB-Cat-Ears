#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define LED_PIN_0 D0
#define LED_PIN_1 D1
#define LED_COUNT_0 22
#define LED_COUNT_1 22
#define SEG_LEN 3

Adafruit_NeoPixel strip0(LED_COUNT_0, LED_PIN_0, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip1(LED_COUNT_1, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

unsigned long lastMoveTime = 0;
const unsigned long idleDelay = 1000; // ms before entering idle
float idleFade = -1.0; // 0 = off, -1 or 1 = full brightness, < 0 for "active" and > 0 for "idle"

void setup() {
  Serial.begin(115200);

  Wire.begin(6, 7); // SDA, SCL for XIAO ESP32-C3
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(10); // threshold in milli gs
  mpu.setMotionDetectionDuration(20); // duration above threshold in ms
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  strip0.begin(); strip0.setBrightness(255); strip0.show();
  strip1.begin(); strip1.setBrightness(255); strip1.show();
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  if (mpu.getMotionInterruptStatus()) {
    lastMoveTime = millis();
    mpu.setMotionInterrupt(true);
  }
  bool isIdle = (millis() - lastMoveTime) > idleDelay; // must not move for idle delay to trigger idle

  float fadeSpeed = 0.01;
  if (isIdle && idleFade < 1.0) idleFade += fadeSpeed;
  if (!isIdle && idleFade > -1.0) idleFade -= 4 * fadeSpeed;
  idleFade = constrain(idleFade, -1.0, 1.0);

  if (idleFade > 0.0) rainbowBreathing(g);
  if (idleFade < 0.0) gyroReactive(g);
}

// ---------------------- ACTIVE MODE ----------------------
void gyroReactive(sensors_event_t &gyro) {
  float rollRate  = gyro.gyro.x * 57.2958;
  float pitchRate = gyro.gyro.y * 57.2958;
  float yawRate   = gyro.gyro.z * 57.2958;

  const float maxRate = 180.0;
  float rotMag = sqrt(gyro.gyro.x * gyro.gyro.x +
                      gyro.gyro.y * gyro.gyro.y +
                      gyro.gyro.z * gyro.gyro.z) * 57.2958;
  rotMag = constrain(rotMag, 0, maxRate);
  uint8_t speed = map(rotMag, 0, maxRate, 100, 50);

  const float maxAxisRate = 180.0;
  auto signedMap = [&](float rate) -> uint8_t {
    rate = constrain(rate, -maxAxisRate, maxAxisRate);
    return (uint8_t)map(rate, -maxAxisRate, maxAxisRate, 0, 255);
  };

  uint8_t r = signedMap(rollRate);
  uint8_t g = signedMap(pitchRate);
  uint8_t b = signedMap(yawRate);

  uint32_t color = increaseSaturation(r, g, b, 1.5);
  // unpack
  r = (color >> 16) & 0xFF;
  g = (color >> 8) & 0xFF;
  b = color & 0xFF;

  uint8_t r1 = b;
  uint8_t g1 = r;
  uint8_t b1 = g;

  r  = constrain(r , 0, 255);
  g  = constrain(g , 0, 255);
  b  = constrain(b , 0, 255);
  r1 = constrain(r1, 0, 255);
  g1 = constrain(g1, 0, 255);
  b1 = constrain(b1, 0, 255);

  // Default to orange when not moving to be more interesting than being white.
  // const float deadband = 10.0;
  // if (fabs(rollRate) < deadband && fabs(pitchRate) < deadband && fabs(yawRate) < deadband) {
  //   r  = 255;
  //   g  = 128;
  //   b  = 0;
  //   r1 = 255;
  //   g1 = 128;
  //   b1 = 0;
  // }

  stripCycle(speed, r, g, b, r1, g1, b1);
}

void stripCycle(uint8_t wait, uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1) {
  float fade = -idleFade;

  for (uint16_t i = 0; i < SEG_LEN; i++) {
    for (uint16_t j = 0; j < strip0.numPixels(); j++) {
      if (j % SEG_LEN != SEG_LEN - i - 1) // invert direction
        strip0.setPixelColor(j, strip0.Color(r0 * fade, g0 * fade, b0 * fade));
      // if (j % SEG_LEN - 1 == SEG_LEN - i - 1) // invert direction
      //   strip0.setPixelColor(j, strip0.Color(0.5 * r * fade, 0.5 * g * fade, 0.5 * b * fade));
      else strip0.setPixelColor(j, 0);
    }

    for (uint16_t j = 0; j < strip1.numPixels(); j++) {
      if (j % SEG_LEN != i) 
        strip1.setPixelColor(j, strip1.Color(r1 * fade, g1 * fade, b1 * fade));
      // if (j % SEG_LEN + 1 == i) 
      //   strip1.setPixelColor(j, strip1.Color(0.5 * b * fade, 0.5 * r * fade, 0.5 * g * fade));
      else strip1.setPixelColor(j, 0);
    }

    strip0.show();
    strip1.show();
    delay(wait);
  }
}

// ---------------------- IDLE MODE ----------------------
void rainbowBreathing(sensors_event_t &gyro) {
  const float maxRate = 30.0;
  float rotMag = sqrt(gyro.gyro.x * gyro.gyro.x +
                      gyro.gyro.y * gyro.gyro.y +
                      gyro.gyro.z * gyro.gyro.z) * 57.2958;
  rotMag = constrain(rotMag, 0, maxRate);

  // Brightness depends on rotation rate (for fun).
  float brightness = fmap(rotMag, 0, maxRate, 0.5, 1.0);
  // Smooth out brightness to stop spikes.
  static float smoothedBrightness = 0.0;
  float target = brightness;
  float alpha = 0.1;
  smoothedBrightness = smoothedBrightness * (1 - alpha) + target * alpha;
  brightness = smoothedBrightness * idleFade;

  // Speed depends on rotation rate (for fun).
  uint8_t speed = map(rotMag, 0, maxRate, 100, 10);

  for (int i = 0; i < strip0.numPixels(); i++) {
    float hue = (float)i / strip0.numPixels() + (millis() % 5000) / 5000.0;
    uint32_t color = hsvToRGB(hue, 1.0, brightness);
    strip0.setPixelColor(i, color);
  }

  // Strip1 gets a reversed animation.
  for (int i = 0; i < strip1.numPixels(); i++) {
    float hue = (float)i / strip1.numPixels() + (millis() % 5000) / 5000.0;
    uint32_t color = hsvToRGB(hue, 1.0, brightness);
    strip1.setPixelColor(strip1.numPixels() - i - 1, color);
  }

  strip0.show();
  strip1.show();

  delay(speed);
}

// Convert HSV → RGB (for rainbow)
uint32_t hsvToRGB(float h, float s, float v) {
  h = fmod(h, 1.0);
  int i = int(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  float r, g, b;
  switch (i % 6) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
  }
  return Adafruit_NeoPixel::Color(r * 255, g * 255, b * 255);
}

uint32_t increaseSaturation(uint8_t r, uint8_t g, uint8_t b, float factor) {
  float rf = r / 255.0;
  float gf = g / 255.0;
  float bf = b / 255.0;

  float maxC = max(rf, max(gf, bf));
  float minC = min(rf, min(gf, bf));
  float delta = maxC - minC;

  float v = maxC; // value
  float s = (maxC == 0) ? 0 : delta / maxC; // saturation

  // Compute Hue.
  float h = 0.0;
  if (delta != 0) {
    if (maxC == rf)      h = fmod((gf - bf) / delta, 6.0);
    else if (maxC == gf) h = ((bf - rf) / delta) + 2.0;
    else                 h = ((rf - gf) / delta) + 4.0;
    h /= 6.0;
    if (h < 0) h += 1.0;
  }

  // Adjust saturation.
  s = constrain(s * factor, 0.0, 1.0);

  return hsvToRGB(h, s, v);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}