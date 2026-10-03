// V4 battlebot controller: ESP32 + Bluetooth gamepad (Bluepad32) + sensors.
//
// Board: "ESP32 Dev Module" from the Bluepad32 board package.
// Libraries: Bluepad32 (board package), ESP32Servo, OneWire, DallasTemperature.
// Drive: 2x DRV8871 (IN1/IN2 PWM), left pair and right pair of JGA25-370 motors.
// Weapon: Hobbywing Skywalker 40A on a 50 Hz servo pulse (1000 us = stop).
// Sensors (see robot_v4/STATUS.md "Sensors" for placement and wiring):
//   GY-521 MPU6050 IMU (I2C) ...... auto upside-down detection + turn-rate limiter
//   A3144 hall + magnet on pulley . weapon rpm ("ready" colour on the pad)
//   2x DS18B20 .................... weapon ESC and front-left drive motor temperature
//   100k/22k divider .............. pack voltage (keeps the ADC below 2.45 V)
// Every sensor is optional: if it is missing or fails, the robot still drives and
// the related feature falls back (e.g. manual inverted toggle with Y).
//
// SAFETY RULES BUILT IN (test every one with the weapon locked before arena use):
//  1. Weapon pulse is 1000 us at boot and whenever no gamepad is connected.
//  2. Failsafe: no gamepad data for FAILSAFE_MS -> drive coast, weapon 1000 us.
//     Sensor code never blocks: I2C read < 1 ms, DS18B20 runs without waiting.
//  3. Weapon needs an arm sequence (hold L1+R1 for 1 s) after every connect.
//  4. Weapon throttle ramps (WEAPON_RAMP_US_PER_S) so spin-up FROM REST stays
//     ~30 A (design_v4.py: 0.64 s to 90 %). The Skywalker has no current limit,
//     and the ramp does NOT protect it after a hit: a rotor knocked to half speed
//     at full throttle can pull ~110 A until it recovers. The slipping round
//     belt is what limits that; measure it on the bench.
//  5. Disarm instantly with the B / Circle button.
//  6. Hot weapon ESC (> ESC_HOT_C) caps weapon throttle at HOT_WEAPON_CAP.
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---- pins (check against the wiring before powering) ----
const int L_IN1 = 25, L_IN2 = 26;   // DRV8871 left
const int R_IN1 = 27, R_IN2 = 14;   // DRV8871 right
const int WEAPON_PIN = 13;          // Skywalker signal; its 5 V BEC feeds ESP32 VIN
const int I2C_SDA = 21, I2C_SCL = 22;
const int HALL_PIN = 32;            // A3144 output: sensor powered from 5 V, open collector pulled up
                                    // to 3.3 V ONLY by an external 10k (never to 5 V)
const int ONEWIRE_PIN = 4;          // DS18B20 data, 4.7k pull-up to 3.3 V
const int VBAT_PIN = 34;            // ADC1 (works while Bluetooth is on)

// ---- control ----
const uint32_t FAILSAFE_MS = 250;
const int WEAPON_MIN_US = 1000, WEAPON_MAX_US = 2000;
const float WEAPON_RAMP_US_PER_S = 1600.0f;  // 0 -> full in 0.63 s, ~30 A peak from rest
const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = (1 << PWM_BITS) - 1;
const int DEADBAND = 40;                     // stick units out of 512

// ---- sensors ----
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB_PER_G = 2048.0f;         // +-16 g range
const float GYRO_LSB_PER_DPS = 16.4f;        // +-2000 dps range
const float INVERT_THRESHOLD_G = 0.5f;       // filtered Z beyond +-0.5 g decides the side
const uint32_t INVERT_HOLD_MS = 300;         // must persist this long (hits are short)
const float YAW_LIMIT_DPS = 690.0f;          // 12 rad/s, below the 14.7 rad/s gyro wheel-lift limit
const uint32_t HALL_MIN_PERIOD_US = 2000;    // ignore pulses faster than 30,000 rpm (noise)
const uint32_t HALL_TIMEOUT_US = 200000;     // no pulse for 0.2 s -> 0 rpm
const float READY_RPM = 10000.0f;            // pad turns green above this
const float VBAT_DIVIDER = (100.0f + 22.0f) / 22.0f;   // 12.6 V -> 2.27 V, 10.5 V -> 1.89 V at the pin
const float VBAT_LOW = 10.5f;                // 3S, warn after VBAT_LOW_MS below this
const float VBAT_OK = 10.9f;                 // clear the warning above this
const uint32_t VBAT_LOW_MS = 2000;           // spin-up sag is shorter than this
const float ESC_HOT_C = 80.0f, MOTOR_HOT_C = 90.0f;
const float HOT_WEAPON_CAP = 0.7f;
// DS18B20s are numbered by their ROM code, not by wiring order. Warm one probe with a finger,
// watch the Serial telemetry, and set which index is taped to the weapon ESC.
const int ESC_TEMP_INDEX = 0;

ControllerPtr pad = nullptr;
Servo weapon;
uint32_t lastData = 0;
bool armed = false;
uint32_t armHoldStart = 0;
float weaponUs = WEAPON_MIN_US;
uint32_t lastLoop = 0;

// inverted handling: automatic from the IMU, manual with Y, back to automatic with X
bool inverted = false;
bool autoInvert = true;
bool imuOk = false;
float azFilt = 1.0f, gzDps = 0.0f;
uint32_t sideChangeSince = 0;

// weapon rpm from the hall sensor
volatile uint32_t hallLastUs = 0, hallPeriodUs = 0;
float weaponRpm = 0.0f;

// temperatures and battery
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature temps(&oneWire);
int nTemps = 0;
DeviceAddress tempAddr[2];                   // looked up once in setup(): no bus search per read
bool batLow = false;
uint32_t batBelowSince = 0;
uint8_t warnedMask = 0;                      // rumble once per warning type per connection
float escC = NAN, motorC = NAN, vbat = NAN;
uint32_t tempRequestedAt = 0;

uint8_t ledState = 255;

void IRAM_ATTR onHall() {
  uint32_t now = micros();
  uint32_t dt = now - hallLastUs;
  if (dt >= HALL_MIN_PERIOD_US) {
    hallPeriodUs = dt;
    hallLastUs = now;
  }
}

void onConnected(ControllerPtr c) {
  if (pad == nullptr) { pad = c; armed = false; lastData = millis(); ledState = 255; warnedMask = 0; }
  else c->disconnect();                      // only one driver
}

void onDisconnected(ControllerPtr c) {
  if (c == pad) { pad = nullptr; armed = false; }
}

void setMotor(int in1, int in2, int cmd) {   // cmd -PWM_MAX..PWM_MAX, 0 = coast
  cmd = constrain(cmd, -PWM_MAX, PWM_MAX);
  if (cmd > 0)      { ledcWrite(in1, cmd); ledcWrite(in2, 0); }
  else if (cmd < 0) { ledcWrite(in1, 0);   ledcWrite(in2, -cmd); }
  else              { ledcWrite(in1, 0);   ledcWrite(in2, 0); }
}

void safeStop() {
  setMotor(L_IN1, L_IN2, 0);
  setMotor(R_IN1, R_IN2, 0);
  armed = false;
  weaponUs = WEAPON_MIN_US;
  weapon.writeMicroseconds(WEAPON_MIN_US);
}

int shape(int v) {                           // stick -512..511 -> -PWM_MAX..PWM_MAX
  if (abs(v) < DEADBAND) return 0;
  return map(v, -512, 511, -PWM_MAX, PWM_MAX);
}

// ---------------------------------------------------------------- IMU
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool imuBegin() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Wire.setTimeOut(5);                        // ms; a broken I2C line must not stall the loop
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x00);                      // wake up
  mpuWrite(0x1A, 0x03);                      // DLPF ~44 Hz
  mpuWrite(0x1B, 0x18);                      // gyro +-2000 dps
  mpuWrite(0x1C, 0x18);                      // accel +-16 g (hits are large)
  return true;
}

void imuUpdate(float dt) {
  if (!imuOk) return;
  // one burst from ACCEL_ZOUT_H (0x3F): AZ, TEMP, GX, GY, GZ = 10 bytes
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3F);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)MPU_ADDR, 10) != 10) return;
  uint8_t b[10];
  for (int i = 0; i < 10; i++) b[i] = Wire.read();
  int16_t az = (int16_t)((b[0] << 8) | b[1]);
  int16_t gz = (int16_t)((b[8] << 8) | b[9]);

  float a = constrain(dt / 0.15f, 0.0f, 1.0f);           // ~0.15 s low-pass rejects hit spikes
  azFilt += a * (az / ACC_LSB_PER_G - azFilt);
  gzDps = gz / GYRO_LSB_PER_DPS;

  if (!autoInvert) return;
  bool wantInverted = inverted;
  if (azFilt < -INVERT_THRESHOLD_G) wantInverted = true;
  else if (azFilt > INVERT_THRESHOLD_G) wantInverted = false;
  if (wantInverted != inverted) {
    if (sideChangeSince == 0) sideChangeSince = millis();
    if (millis() - sideChangeSince > INVERT_HOLD_MS) { inverted = wantInverted; sideChangeSince = 0; }
  } else {
    sideChangeSince = 0;
  }
}

// ---------------------------------------------------------------- other sensors
void rpmUpdate() {
  noInterrupts();
  uint32_t last = hallLastUs, period = hallPeriodUs;
  interrupts();
  weaponRpm = (period == 0 || micros() - last > HALL_TIMEOUT_US) ? 0.0f : 60.0e6f / period;
}

void slowSensors() {                         // temperatures and battery, ~1 Hz, non-blocking
  uint32_t now = millis();
  if (nTemps > 0) {
    if (tempRequestedAt == 0) {
      temps.requestTemperatures();           // returns at once (setWaitForConversion(false))
      tempRequestedAt = now;
    } else if (now - tempRequestedAt > 250) {  // 10-bit conversion takes 188 ms
      float a = temps.getTempC(tempAddr[ESC_TEMP_INDEX]);
      float b = nTemps > 1 ? temps.getTempC(tempAddr[1 - ESC_TEMP_INDEX]) : DEVICE_DISCONNECTED_C;
      escC = (a == DEVICE_DISCONNECTED_C) ? NAN : a;
      motorC = (b == DEVICE_DISCONNECTED_C) ? NAN : b;
      tempRequestedAt = 0;
    }
  }
  static uint32_t lastV = 0;
  if (now - lastV >= 100) {                  // 10 Hz, lightly filtered so motor surges do not flicker the warning
    lastV = now;
    float v = analogReadMilliVolts(VBAT_PIN) / 1000.0f * VBAT_DIVIDER;
    vbat = isnan(vbat) ? v : vbat + 0.2f * (v - vbat);
    if (vbat > 5.0f && vbat < VBAT_LOW) {      // >5 V: ignore USB-only power on the bench
      if (batBelowSince == 0) batBelowSince = now;
      if (now - batBelowSince > VBAT_LOW_MS) batLow = true;
    } else {
      batBelowSince = 0;
      if (vbat > VBAT_OK) batLow = false;
    }
  }
}

bool escHot() { return !isnan(escC) && escC > ESC_HOT_C; }
bool motorHot() { return !isnan(motorC) && motorC > MOTOR_HOT_C; }
bool batteryLow() { return batLow; }

// pad light bar: blue = safe, yellow = armed, green = weapon at speed,
// purple = something hot, red = battery low. Only sent when it changes.
void updatePadFeedback() {
  uint8_t s;
  if (batteryLow()) s = 4;
  else if (escHot() || motorHot()) s = 3;
  else if (!armed) s = 0;
  else if (weaponRpm >= READY_RPM) s = 2;
  else s = 1;
  if (s == ledState) return;
  static const uint8_t col[5][3] = {{0, 0, 255}, {255, 160, 0}, {0, 255, 0}, {160, 0, 255}, {255, 0, 0}};
  pad->setColorLED(col[s][0], col[s][1], col[s][2]);
  if (s >= 3 && !(warnedMask & (1 << s))) {   // one buzz per warning type per connection
    pad->playDualRumble(0, 300, 0x80, 0x40);
    warnedMask |= 1 << s;
  }
  ledState = s;
}

void telemetry() {                           // bench data over USB serial, 2 Hz
  static uint32_t last = 0;
  if (millis() - last < 500) return;
  last = millis();
  Serial.printf("pad=%d armed=%d inv=%d(%s) az=%.2f gz=%.0fdps rpm=%.0f vbat=%.2f esc=%.1fC motor=%.1fC weapon_us=%.0f\n",
                pad != nullptr, armed, inverted, autoInvert ? "auto" : "manual", azFilt, gzDps,
                weaponRpm, vbat, escC, motorC, weaponUs);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  weapon.setPeriodHertz(50);
  weapon.attach(WEAPON_PIN, WEAPON_MIN_US, WEAPON_MAX_US);
  weapon.writeMicroseconds(WEAPON_MIN_US);  // first thing: weapon stopped
  for (int p : {L_IN1, L_IN2, R_IN1, R_IN2}) ledcAttach(p, PWM_FREQ, PWM_BITS);
  safeStop();

  imuOk = imuBegin();
  autoInvert = imuOk;
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHall, FALLING);
  temps.begin();
  nTemps = min((int)temps.getDeviceCount(), 2);
  for (int i = 0; i < nTemps; i++) {
    if (!temps.getAddress(tempAddr[i], i)) { nTemps = i; break; }
  }
  temps.setResolution(10);
  temps.setWaitForConversion(false);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  Serial.printf("IMU %s, %d temperature sensor(s)\n", imuOk ? "OK" : "MISSING (manual invert with Y)", nTemps);

  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();                // pair fresh each power-up
  lastLoop = millis();
}

void loop() {
  bool fresh = BP32.update();
  uint32_t now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  imuUpdate(dt);
  rpmUpdate();
  slowSensors();
  telemetry();

  if (pad && pad->isConnected() && fresh && pad->hasData()) lastData = now;
  if (!pad || !pad->isConnected() || now - lastData > FAILSAFE_MS) {
    safeStop();
    delay(5);
    return;
  }

  // ---- arming / disarming ----
  bool l1 = pad->l1(), r1 = pad->r1();
  if (pad->b()) { armed = false; }
  if (!armed && l1 && r1) {
    if (armHoldStart == 0) armHoldStart = now;
    if (now - armHoldStart > 1000) armed = true;
  } else {
    armHoldStart = 0;
  }
  // Y: manual flip (turns automatic mode off). X: back to automatic (needs the IMU).
  static bool lastY = false;
  if (pad->y() && !lastY) { inverted = !inverted; autoInvert = false; }
  lastY = pad->y();
  if (pad->x() && imuOk) autoInvert = true;

  // ---- drive: left stick throttle, right stick steering (arcade) ----
  // Upside down (rolled over) only the throttle sign changes; the side swap and the
  // reversed wheel direction cancel for steering.
  int thr = -shape(pad->axisY());
  int str = shape(pad->axisRX());
  if (inverted) thr = -thr;
  // Turn-rate limiter: with the weapon spinning, gyroscopic torque lifts a wheel above
  // ~14.7 rad/s (design_v4.py). Scale steering down once the IMU sees more than YAW_LIMIT_DPS.
  if (imuOk && weaponRpm > 3000.0f && fabsf(gzDps) > YAW_LIMIT_DPS) {
    str = (int)(str * YAW_LIMIT_DPS / fabsf(gzDps));
  }
  setMotor(L_IN1, L_IN2, thr + str);
  setMotor(R_IN1, R_IN2, thr - str);

  // ---- weapon: right trigger 0..1023, ramp limited up, instant down ----
  float cap = escHot() ? HOT_WEAPON_CAP : 1.0f;
  float target = WEAPON_MIN_US;
  if (armed) target = WEAPON_MIN_US + (WEAPON_MAX_US - WEAPON_MIN_US) * cap * (pad->throttle() / 1023.0f);
  if (target > weaponUs) weaponUs = min(target, weaponUs + WEAPON_RAMP_US_PER_S * dt);
  else weaponUs = target;
  weapon.writeMicroseconds((int)weaponUs);

  updatePadFeedback();
  delay(5);
}
