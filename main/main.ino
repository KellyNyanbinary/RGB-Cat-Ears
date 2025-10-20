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
const unsigned long idleDelay = 5000;  // ms before entering idle

void setup() {
  Serial.begin(115200);

  Wire.begin(6, 7);  // SDA, SCL for XIAO ESP32-C3
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(10);  // threshold in milli gs
  mpu.setMotionDetectionDuration(20);   // duration above threshold in ms
  mpu.setInterruptPinLatch(true);       // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  strip0.begin();
  strip0.setBrightness(255);
  strip0.show();
  strip1.begin();
  strip1.setBrightness(255);
  strip1.show();
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  if (mpu.getMotionInterruptStatus()) {
    lastMoveTime = millis();
    mpu.setMotionInterrupt(true);
  }

  bool isIdle = (millis() - lastMoveTime) > idleDelay;  // must not move for idle delay to trigger idle
  if (isIdle) rainbowBreathing(g);
  else gyroReactive(g);
}

// ---------------------- ACTIVE MODE ----------------------
void gyroReactive(sensors_event_t &gyro) {
  float rollRate = gyro.gyro.x * 57.2958f;
  float pitchRate = gyro.gyro.y * 57.2958f;
  float yawRate = gyro.gyro.z * 57.2958f;

  const float maxRate = 180.0f;
  float rotMag = sqrt(gyro.gyro.x * gyro.gyro.x + gyro.gyro.y * gyro.gyro.y + gyro.gyro.z * gyro.gyro.z) * 57.2958f;
  rotMag = constrain(rotMag, 0, maxRate);
  uint8_t speed = map(rotMag, 0, maxRate, 100, 50);

  const float maxAxisRate = 180.0f;
  auto signedMap = [&](float rate) -> uint8_t {
    rate = constrain(rate, -maxAxisRate, maxAxisRate);
    return (uint8_t)map(rate, -maxAxisRate, maxAxisRate, 0, 255);
  };

  uint8_t r = signedMap(rollRate);
  uint8_t g = signedMap(pitchRate);
  uint8_t b = signedMap(yawRate);

  uint32_t color = increaseSaturation(r, g, b, 1.5f);
  // unpack
  r = (color >> 16) & 0xFF;
  g = (color >> 8) & 0xFF;
  b = color & 0xFF;

  uint8_t r1 = b;
  uint8_t g1 = r;
  uint8_t b1 = g;

  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
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
  for (uint16_t i = 0; i < strip0.numPixels() - 1; i++) {
    strip0.setPixelColor(i, strip0.getPixelColor(i + 1));
  }

  // Invert direction for the other ear.
  for (uint16_t i = strip1.numPixels() - 1; i > 0; i--) {
    strip1.setPixelColor(i, strip1.getPixelColor(i - 1));
  }

  strip0.setPixelColor(strip0.numPixels() - 1, strip0.Color(r0, g0, b0));
  strip1.setPixelColor(0, strip1.Color(r1, g1, b1));

  strip0.show();
  strip1.show();
  delay(wait);
}

// ---------------------- IDLE MODE ----------------------
void rainbowBreathing(sensors_event_t &gyro) {
  const float maxRate = 30.0;
  float rotMag =
    sqrt(
      gyro.gyro.x * gyro.gyro.x + gyro.gyro.y * gyro.gyro.y + gyro.gyro.z * gyro.gyro.z)
    * 57.2958;
  rotMag = constrain(rotMag, 0, maxRate);

  // Brightness depends on rotation rate (for fun).
  float brightness = fmap(rotMag, 0, maxRate, 0.5f, 1.0f);
  // Smooth out brightness to stop spikes.
  static float smoothedBrightness = 0.0f;
  float target = brightness;
  float alpha = 0.1f;
  smoothedBrightness = smoothedBrightness * (1 - alpha) + target * alpha;
  brightness = smoothedBrightness;

  for (uint16_t i = 0; i < strip0.numPixels() - 1; i++) {
    strip0.setPixelColor(i, strip0.getPixelColor(i + 1));
  }

  // Invert direction for the other ear.
  for (uint16_t i = strip1.numPixels() - 1; i > 0; i--) {
    strip1.setPixelColor(i, strip1.getPixelColor(i - 1));
  }

  float hue = (millis() % 5000) / 5000.0f;
  uint32_t color = Adafruit_NeoPixel::ColorHSV(hue * 65535, 255, brightness * 255);
  strip0.setPixelColor(strip0.numPixels() - 1, color);
  strip1.setPixelColor(0, color);

  strip0.show();
  strip1.show();
  delay(100);
}

/**
 * @brief Increase the saturation of the color passed in by the factor.
 *
 * @param r The amount of red in [0, 255]
 * @param g The amount of green in [0, 255]
 * @param b The amount of blue in [0, 255]
 * @param factor The factor to multiply the saturation by
 * @return The resulting color in a packed RGB.
 */
uint32_t increaseSaturation(uint8_t r, uint8_t g, uint8_t b, float factor) {
  float rf = r / 255.0f;
  float gf = g / 255.0f;
  float bf = b / 255.0f;

  float maxC = max(rf, max(gf, bf));
  float minC = min(rf, min(gf, bf));
  float delta = maxC - minC;

  float v = maxC;                            // value
  float s = (maxC == 0) ? 0 : delta / maxC;  // saturation

  // Compute Hue.
  float h = 0.0f;
  if (delta != 0) {
    if (maxC == rf) h = fmod((gf - bf) / delta, 6.0f);
    else if (maxC == gf) h = ((bf - rf) / delta) + 2.0f;
    else h = ((rf - gf) / delta) + 4.0f;
    h /= 6.0f;
    if (h < 0) h += 1.0f;
  }

  // Adjust saturation.
  s = constrain(s * factor, 0.0f, 1.0f);

  return Adafruit_NeoPixel::ColorHSV(h * 65535, s * 255, v * 255);
}

/**
 * @brief Map input float from the input range to the output range
 *
 * @param x The input to map
 * @param in_min The minimum of the mapping range, but lesser values are mapped
 *               anways
 * @param in_max The maximum of the mapping range, but greater values are 
 *               mapped anyways
 * @param out_min The minimum of the output range, lesser outputs are possible
 *                when the input is less than in_min
 * @param out_max The maximum of the output range, greater outputs are possible
 *                when the input is greater than in_max
 * @return The mapped to value
 */
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}