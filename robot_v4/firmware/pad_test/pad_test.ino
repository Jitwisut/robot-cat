// Gamepad bench test for V4: run this on the bare ESP32 (no motors, no ESC)
// before buying the rest of the parts.
// Board: "ESP32 Dev Module" from the Bluepad32 board package. Serial Monitor 115200.
//
// It checks two things the main firmware depends on:
//  1. The pad pairs with Bluepad32 at all (cheap PS4 clones do not always).
//  2. The pad keeps sending reports while the sticks are held still. The main
//     firmware's failsafe stops the robot after 250 ms without a new report,
//     so "max gap" below must stay well under 250 ms with the sticks untouched.
#include <Bluepad32.h>

ControllerPtr pad = nullptr;
uint32_t lastReport = 0, maxGap = 0, reports = 0, windowStart = 0;

void onConnected(ControllerPtr c) {
  pad = c;
  ControllerProperties p = c->getProperties();
  Serial.printf("CONNECTED model=%s vid=0x%04x pid=0x%04x\n", c->getModelName().c_str(), p.vendor_id, p.product_id);
  lastReport = windowStart = millis();
  maxGap = reports = 0;
}

void onDisconnected(ControllerPtr c) {
  if (c == pad) pad = nullptr;
  Serial.println("DISCONNECTED");
}

void setup() {
  Serial.begin(115200);
  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();
  Serial.println("Put the pad in pairing mode (PS4: hold SHARE + PS until the light flashes).");
}

void loop() {
  bool fresh = BP32.update();
  uint32_t now = millis();
  if (pad && pad->isConnected() && fresh && pad->hasData()) {
    maxGap = max(maxGap, now - lastReport);
    lastReport = now;
    reports++;
  }
  if (pad && now - windowStart >= 1000) {
    Serial.printf("reports/s=%lu  max_gap_ms=%lu  LX=%d LY=%d RX=%d RY=%d R2=%d L1=%d R1=%d B=%d Y=%d  %s\n",
                  reports, maxGap, pad->axisX(), pad->axisY(), pad->axisRX(), pad->axisRY(),
                  pad->throttle(), pad->l1(), pad->r1(), pad->b(), pad->y(),
                  maxGap < 100 ? "OK" : "GAP TOO LONG - failsafe would trip");
    reports = 0; maxGap = 0; windowStart = now;
  }
  delay(2);
}
