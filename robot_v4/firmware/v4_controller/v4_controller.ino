// V4 battlebot controller: ESP32 + Bluetooth gamepad (Bluepad32).
//
// Board: "ESP32 Dev Module" from the Bluepad32 board package
//        (Arduino IDE -> Boards Manager URL from github.com/ricardoquesada/esp32-arduino-lib-builder / bluepad32).
// Drive: 2x DRV8871 (IN1/IN2 PWM), left pair and right pair of JGA25-370 motors.
// Weapon: Hobbywing Skywalker 40A on a 50 Hz servo pulse (1000 us = stop).
//
// SAFETY RULES BUILT IN (test every one with the weapon locked before arena use):
//  1. Weapon pulse is 1000 us at boot and whenever no gamepad is connected.
//  2. Failsafe: no gamepad data for FAILSAFE_MS -> drive coast, weapon 1000 us.
//  3. Weapon needs an arm sequence (hold L1+R1 for 1 s) after every connect.
//  4. Weapon throttle ramps (WEAPON_RAMP_US_PER_S) so spin-up FROM REST stays
//     ~30 A (design_v4.py: 0.64 s to 90 %). The Skywalker has no current limit,
//     and the ramp does NOT protect it after a hit: a rotor knocked to half speed
//     at full throttle can pull ~110 A until it recovers. The slipping round
//     belt is what limits that; measure it on the bench.
//  5. Disarm instantly with the B / Circle button.
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ---- pins (check against the wiring before powering) ----
const int L_IN1 = 25, L_IN2 = 26;   // DRV8871 left
const int R_IN1 = 27, R_IN2 = 14;   // DRV8871 right
const int WEAPON_PIN = 13;          // Skywalker signal; its 5 V BEC feeds ESP32 VIN

const uint32_t FAILSAFE_MS = 250;
const int WEAPON_MIN_US = 1000, WEAPON_MAX_US = 2000;
const float WEAPON_RAMP_US_PER_S = 1600.0f;  // 0 -> full in 0.63 s, ~30 A peak from rest
const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = (1 << PWM_BITS) - 1;
const int DEADBAND = 40;                     // stick units out of 512

ControllerPtr pad = nullptr;
Servo weapon;
uint32_t lastData = 0;
bool armed = false;
uint32_t armHoldStart = 0;
float weaponUs = WEAPON_MIN_US;
bool inverted = false;                       // toggled with Y / Triangle when the robot is upside down
uint32_t lastLoop = 0;

void onConnected(ControllerPtr c) {
  if (pad == nullptr) { pad = c; armed = false; lastData = millis(); }
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

void setup() {
  Serial.begin(115200);
  weapon.setPeriodHertz(50);
  weapon.attach(WEAPON_PIN, WEAPON_MIN_US, WEAPON_MAX_US);
  weapon.writeMicroseconds(WEAPON_MIN_US);  // first thing: weapon stopped
  for (int p : {L_IN1, L_IN2, R_IN1, R_IN2}) ledcAttach(p, PWM_FREQ, PWM_BITS);
  safeStop();
  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();                // pair fresh each power-up
  lastLoop = millis();
}

void loop() {
  bool fresh = BP32.update();
  uint32_t now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

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
  static bool lastY = false;
  if (pad->y() && !lastY) inverted = !inverted;
  lastY = pad->y();

  // ---- drive: left stick throttle, right stick steering (arcade) ----
  int thr = -shape(pad->axisY());
  int str = shape(pad->axisRX());
  if (inverted) thr = -thr;
  setMotor(L_IN1, L_IN2, thr + str);
  setMotor(R_IN1, R_IN2, thr - str);

  // ---- weapon: right trigger 0..1023, ramp limited up, instant down ----
  float target = WEAPON_MIN_US;
  if (armed) target = WEAPON_MIN_US + (WEAPON_MAX_US - WEAPON_MIN_US) * (pad->throttle() / 1023.0f);
  if (target > weaponUs) weaponUs = min(target, weaponUs + WEAPON_RAMP_US_PER_S * dt);
  else weaponUs = target;
  weapon.writeMicroseconds((int)weaponUs);

  delay(5);
}
