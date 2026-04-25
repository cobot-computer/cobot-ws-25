#include <SoftwareSerial.h>
#include <Tic.h>

SoftwareSerial ticSerial(10, 11); // RX, TX
TicSerial tic(ticSerial);

unsigned long prevTime = 0;

void driveFor(int32_t velocity, unsigned long ms) {
  tic.exitSafeStart();
  tic.setTargetVelocity(velocity);
  unsigned long start = millis();
  while (millis() - start < ms) {
    tic.resetCommandTimeout();
    delay(10);
  }
  tic.setTargetVelocity(0);
  tic.resetCommandTimeout();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Tic control t834...");

  ticSerial.begin(9600);
  delay(20);

  tic.energize();
  delay(20);
  tic.exitSafeStart();

  tic.setMaxSpeed(1000000);
  tic.setStartingSpeed(0);
  tic.setMaxAccel(1000000);
  tic.setMaxDecel(1000000);
}

void loop() {
  
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String cmd2;
    if (cmd == "open") {
      driveFor(-1000000, 1500);
      Serial.println("ack open");
    } else if (cmd == "close") {
      Serial.println("ack close");
      while (cmd == "close")
      {
        tic.setTargetVelocity(500000);
        if (cmd2 == "release")
          {
            tic.setTargetVelocity(0);
            Serial.println("ack release");
            return;
          }
        if (millis() - prevTime >= 500)
        {
          cmd2 = Serial.readStringUntil('\n');
          cmd2.trim();
          prevTime = millis();
          if (cmd2 == "release")
          {
            tic.setTargetVelocity(0);
            Serial.println("ack release");
            return;
          }
        }
      }
    }
  }
}
