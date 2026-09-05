#include <Servo.h>

Servo panServo;
Servo tiltServo;

// ---------------- Pins ----------------
const int PAN_PIN   = 10;
const int TILT_PIN  = 9;
const int LASER_PIN = 8;    // Change if your laser is on another pin

String input = "";

// --------------------------------------------------
void setup() {
  Serial.begin(115200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // Home position
  panServo.write(87);
  tiltServo.write(80);

  delay(1000);

  Serial.println("Turret Ready");
}

// --------------------------------------------------
void loop() {
//
//  while (Serial.available()) {
//
//    char c = Serial.read();
//
//    if (c == '\n' || c == '\r') {
//
//      if (input.length() > 0) {
//        processCommand(input);
//        input = "";
//      }
//
//    } else {
//      input += c;
//    }
//  }
}

// --------------------------------------------------
void processCommand(String cmd) {

  cmd.trim();

  // --------------------------------------
  // Format 1:
  // PAN:-40,TILT:20
  // (Manual parsing — Arduino Uno's AVR sscanf does NOT support %f,
  //  so we parse the string ourselves instead of using sscanf.)
  // --------------------------------------
  if (cmd.startsWith("PAN:")) {

    int commaIdx = cmd.indexOf(',');
    int colon1   = cmd.indexOf(':');              // colon after "PAN"
    int colon2   = cmd.indexOf(':', commaIdx);    // colon after "TILT"

    if (commaIdx > 0 && colon1 > 0 && colon2 > 0) {

      float panDeg  = cmd.substring(colon1 + 1, commaIdx).toFloat();
      float tiltDeg = cmd.substring(colon2 + 1).toFloat();

      moveTurret(panDeg, tiltDeg);

    } else {
      Serial.println("Invalid PAN/TILT command");
    }

    return;
  }

  // --------------------------------------
  // Format 2:
  // P90
  // --------------------------------------
  if (cmd.startsWith("P")) {

    int value = cmd.substring(1).toInt();
    value = constrain(value, 0, 174);

    panServo.write(value);

    Serial.print("PAN=");
    Serial.println(value);

    return;
  }

  // --------------------------------------
  // Format 3:
  // T120
  // --------------------------------------
  if (cmd.startsWith("T")) {

    int value = cmd.substring(1).toInt();
    value = constrain(value, 0, 180);

    tiltServo.write(value);

    Serial.print("TILT=");
    Serial.println(value);

    return;
  }

  // --------------------------------------
  // Format 4:
  // L1 / L0
  // --------------------------------------
  if (cmd.startsWith("L")) {

    if (cmd.length() >= 2) {

      bool state = (cmd.charAt(1) == '1');

      digitalWrite(LASER_PIN, state);

      Serial.print("LASER=");
      Serial.println(state ? "ON" : "OFF");
    }

    return;
  }

  Serial.println("Unknown command");
}

// --------------------------------------------------
void moveTurret(float panDeg, float tiltDeg) {

  // PAN mapping (re-derived for 240°-rated servo, 0.75 write-units/deg):
  // Center write = 87 (= 0°)
  // Usable write range = 0 .. 174 (symmetric ±87 around center)
  // Physical degrees = write_units * (240/180) => 87 write-units = ~116°
  // +116° -> write 0   (left max)
  //    0° -> write 87  (center)
  // -116° -> write 174 (right max)
  int panWrite = round(87.0 - 0.75 * panDeg);
  panWrite = constrain(panWrite, 0, 174);

  // TILT mapping (unchanged — update if tilt limits are re-measured):
  // 0° -> 80

  int tiltWrite = round(80.0 + tiltDeg);
  tiltWrite = constrain(tiltWrite, 0, 180);

  panServo.write(panWrite);
  tiltServo.write(tiltWrite);

  Serial.print("PAN=");
  Serial.print(panWrite);

  Serial.print("  TILT=");
  Serial.println(tiltWrite);
}