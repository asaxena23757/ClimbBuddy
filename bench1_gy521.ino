// Bench test 1: GY-521 (MPU-6050) on its own.
//
// Wiring (Uno):  VCC -> 5V   GND -> GND   SDA -> A4   SCL -> A5
// Open the Serial Monitor at 115200 baud (line ending doesn't matter).
//
// Stage 1: scans the I2C bus and checks the chip identity.
// Stage 2: "keep it still" test with PASS / CHECK results.
// Stage 3: live data. Pick the board up and tilt it, then drop it a short
//          way onto a cushion to see FREEFALL / IMPACT flagged.
#include <Wire.h>

static uint8_t addr = 0;

static void printHex(uint8_t v) {
  char b[8];
  snprintf(b, sizeof(b), "0x%02X", v);
  Serial.print(b);
}

static bool probe(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t a, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(a, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool writeReg(uint8_t a, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

// Returns false if the sensor can't be found yet (the caller keeps retrying).
static bool findSensor() {
  Serial.println(F("Stage 1: scanning the I2C bus (SDA = A4, SCL = A5)..."));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (probe(a)) {
      Serial.print(F("  found a device at "));
      printHex(a);
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  No I2C devices found. Check, in this order:"));
    Serial.println(F("   1. GY-521 VCC -> 5V and GND -> GND (is its red LED on?)"));
    Serial.println(F("   2. SDA -> A4 and SCL -> A5 (not swapped)"));
    Serial.println(F("   3. Uno GND is connected to the breadboard ground rail"));
    Serial.println(F("   4. Header pins are soldered/seated properly on the GY-521"));
    Serial.println(F("  Retrying in 2 s..."));
    return false;
  }
  const uint8_t candidates[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    const uint8_t a = candidates[i];
    if (!probe(a)) continue;
    uint8_t who = 0;
    if (!readRegs(a, 0x75, &who, 1)) continue;
    Serial.print(F("  WHO_AM_I at "));
    printHex(a);
    Serial.print(F(" = "));
    printHex(who);
    if (who == 0x68 || who == 0x70 || who == 0x72 || who == 0x98) {
      Serial.println(F("  -> MPU-6050 (or compatible clone). Good."));
      addr = a;
      return true;
    }
    if (who == 0x71 || who == 0x73) {
      Serial.println(F("  -> MPU-9250/9255. Works for this project (same accel/gyro registers)."));
      addr = a;
      return true;
    }
    Serial.println(F("  -> not an MPU-6050. (A DS3231 clock also answers at 0x68.)"));
  }
  Serial.println(F("  Devices were found, but none at 0x68/0x69 that look like an MPU-6050."));
  Serial.println(F("  Retrying in 2 s..."));
  return false;
}

static void initSensor() {
  writeReg(addr, 0x6B, 0x01);  // wake, clock from gyro PLL
  writeReg(addr, 0x1A, 0x02);  // low-pass ~94 Hz
  writeReg(addr, 0x19, 9);     // 100 Hz sample rate
  writeReg(addr, 0x1B, 0x08);  // gyro +/-500 deg/s
  writeReg(addr, 0x1C, 0x18);  // accel +/-16 g
  delay(100);
}

static bool readSample(float* a, float* g) {
  uint8_t r[14];
  if (!readRegs(addr, 0x3B, r, 14)) return false;
  int16_t v[7];
  for (uint8_t i = 0; i < 7; i++) v[i] = (int16_t)(((uint16_t)r[2 * i] << 8) | r[2 * i + 1]);
  a[0] = v[0] / 2048.0f;
  a[1] = v[1] / 2048.0f;
  a[2] = v[2] / 2048.0f;
  g[0] = v[4] / 65.5f;
  g[1] = v[5] / 65.5f;
  g[2] = v[6] / 65.5f;
  return true;
}

static void stillTest() {
  Serial.println(F("Stage 2: lay the board FLAT and keep it STILL."));
  for (uint8_t s = 3; s > 0; s--) {
    Serial.print(F("  starting in "));
    Serial.println(s);
    delay(1000);
  }
  Serial.println(F("  measuring for 2 seconds..."));
  float sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0}, sm = 0, smm = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < 200; i++) {
    float a[3], g[3];
    if (readSample(a, g)) {
      const float m = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      for (uint8_t k = 0; k < 3; k++) {
        sa[k] += a[k];
        sg[k] += g[k];
      }
      sm += m;
      smm += m * m;
      n++;
    }
    delay(10);
  }
  if (n < 150) {
    Serial.print(F("  RESULT: FAIL - only "));
    Serial.print(n);
    Serial.println(F(" of 200 reads worked. Loose wire? Try shorter jumpers or reseat SDA/SCL."));
    return;
  }
  const float mm = sm / n;
  const float var = smm / n - mm * mm;
  const float sd = sqrtf(var > 0 ? var : 0);
  Serial.print(F("  mean accel (g):  x="));
  Serial.print(sa[0] / n, 3);
  Serial.print(F("  y="));
  Serial.print(sa[1] / n, 3);
  Serial.print(F("  z="));
  Serial.println(sa[2] / n, 3);
  Serial.print(F("  mean |a| = "));
  Serial.print(mm, 3);
  Serial.print(F(" g   noise (std) = "));
  Serial.print(sd, 4);
  Serial.println(F(" g"));
  Serial.print(F("  gyro offset (deg/s):  x="));
  Serial.print(sg[0] / n, 2);
  Serial.print(F("  y="));
  Serial.print(sg[1] / n, 2);
  Serial.print(F("  z="));
  Serial.println(sg[2] / n, 2);

  bool ok = true;
  if (mm < 0.05f) {
    ok = false;
    Serial.println(F("  FAIL: reads ~0 g. Sensor asleep or badly powered; check VCC/GND, reseat, reset."));
  } else if (mm < 0.95f || mm > 1.05f) {
    ok = false;
    Serial.println(F("  CHECK: |a| should be about 1.00 g at rest. Was the board really still?"));
  } else {
    Serial.println(F("  PASS: |a| is about 1 g."));
  }
  if (sd > 0.03f) {
    ok = false;
    Serial.println(F("  CHECK: too noisy at rest. Was it moving? If not, check power and wire length."));
  } else if (mm >= 0.05f) {
    Serial.println(F("  PASS: noise is low."));
  }
  bool gyroOk = true;
  for (uint8_t k = 0; k < 3; k++) {
    const float o = sg[k] / n;
    if (o > 20.0f || o < -20.0f) gyroOk = false;
  }
  if (!gyroOk) {
    ok = false;
    Serial.println(F("  CHECK: gyro offset is over 20 deg/s at rest (was the board really still?)."));
  } else if (mm >= 0.05f) {
    Serial.println(F("  PASS: gyro offsets are small."));
  }
  uint8_t up = 0;
  for (uint8_t k = 1; k < 3; k++) {
    const float c = sa[k] / n, b = sa[up] / n;
    if ((c < 0 ? -c : c) > (b < 0 ? -b : b)) up = k;
  }
  Serial.print(F("  gravity is along the "));
  Serial.print(up == 0 ? 'X' : (up == 1 ? 'Y' : 'Z'));
  Serial.println(F(" axis (the sketch doesn't care which way the board is mounted)."));
  Serial.println(ok ? F("  RESULT: PASS") : F("  RESULT: see the lines above"));
}

static uint32_t nextSampleMs = 0, nextPrintMs = 0;
static float winMin = 99, winMax = 0, lastA[3] = {0, 0, 0}, lastG[3] = {0, 0, 0}, lastMag = 0;
static uint16_t readErrors = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);  // don't hang forever on a loose wire
#endif
  Serial.println(F("=== Bench test 1: GY-521 (MPU-6050) ==="));
  while (!findSensor()) delay(2000);
  initSensor();
  Serial.print(F("Using the sensor at "));
  printHex(addr);
  Serial.println();
  stillTest();
  Serial.println(F("Stage 3: live data (10 lines/s). Tilt it, shake it, then drop it ~30 cm onto a cushion."));
  Serial.println(F("  Free fall reads |a| near 0 g (<< FREEFALL), the landing is a spike (>> IMPACT)."));
  nextSampleMs = nextPrintMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if ((int32_t)(now - nextSampleMs) >= 0) {
    nextSampleMs += 10;
    if ((int32_t)(now - nextSampleMs) > 100) nextSampleMs = now + 10;
    if (readSample(lastA, lastG)) {
      lastMag = sqrtf(lastA[0] * lastA[0] + lastA[1] * lastA[1] + lastA[2] * lastA[2]);
      if (lastMag < winMin) winMin = lastMag;
      if (lastMag > winMax) winMax = lastMag;
    } else {
      readErrors++;
    }
  }
  if ((int32_t)(now - nextPrintMs) >= 0) {
    nextPrintMs += 100;
    if ((int32_t)(now - nextPrintMs) > 1000) nextPrintMs = now + 100;
    if (readErrors) {
      Serial.print(F("I2C read errors in the last 100 ms: "));
      Serial.println(readErrors);
      readErrors = 0;
    }
    Serial.print(F("ax:"));  Serial.print(lastA[0], 2);
    Serial.print(F(" ay:")); Serial.print(lastA[1], 2);
    Serial.print(F(" az:")); Serial.print(lastA[2], 2);
    Serial.print(F(" |a|:")); Serial.print(lastMag, 2);
    Serial.print(F("  gx:")); Serial.print(lastG[0], 0);
    Serial.print(F(" gy:")); Serial.print(lastG[1], 0);
    Serial.print(F(" gz:")); Serial.print(lastG[2], 0);
    if (winMin < 0.35f) {
      Serial.print(F("   << FREEFALL (min "));
      Serial.print(winMin, 2);
      Serial.print(F(" g)"));
    }
    if (winMax > 2.0f) {
      Serial.print(F("   >> IMPACT (max "));
      Serial.print(winMax, 1);
      Serial.print(F(" g)"));
    }
    Serial.println();
    winMin = 99;
    winMax = 0;
  }
}
