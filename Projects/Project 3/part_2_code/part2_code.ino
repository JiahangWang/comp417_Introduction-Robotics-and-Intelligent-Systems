// Jiahang Wang

// I used this code to print out the direction information I needed for data analysis.
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>

Madgwick filter;
float roll, pitch, yaw;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized!");
  filter.begin(50); // update rate of the Madgwick filter
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Pass data to the Madgwick filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // Get the current orientation
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    // Print the orientation data
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Yaw: ");
    Serial.println(yaw);

    // Print the gyroscope data
    Serial.print("Gyro X: ");
    Serial.print(gx);
    Serial.print(" Gyro Y: ");
    Serial.print(gy);
    Serial.print(" Gyro Z: ");
    Serial.println(gz);
  }

  delay(100);
}
