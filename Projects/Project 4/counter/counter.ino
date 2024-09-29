// Jiahang Wang(261011319)
// This is the program for step counting and blink
#include <Arduino_LSM6DS3.h>

const int ledPin = 13;  
int stepCount = 0;
bool highThresholdCrossed = false;
bool stepDetected = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(ledPin, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  float x, y, z;
  // change the threshold setting at here:
  float starting = 1.65;
  float ending = 0.6;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    float totalAcceleration = sqrt(x * x + y * y + z * z);

    // check step
    if (!highThresholdCrossed && totalAcceleration > starting) {
      highThresholdCrossed = true;
    }

    if (highThresholdCrossed && !stepDetected && totalAcceleration < ending) {
      stepCount++;
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);

      // print and blink
      Serial.print("blink ");
      Serial.println(stepCount);
      
      stepDetected = true;
    }

    // reset
    if (stepDetected && totalAcceleration > ending && totalAcceleration < starting) {
      highThresholdCrossed = false;
      stepDetected = false;
    }
  }
}
