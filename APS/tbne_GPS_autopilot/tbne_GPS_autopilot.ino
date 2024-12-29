// LIBRARIES ---------------------------------------------------------------------
#include <Wire.h>                                                                 // I2C library
#include <Adafruit_HMC5883_U.h>                                                   // Compass library
#include <TinyGPS++.h>                                                            // Neo6M GPS library
#include <SoftwareSerial.h>                                                       // UART library
#include <ESP32Servo.h>                                                           // Servo library
#include "macros.h"

// GPS and Compass ---------------------------------------------------------------
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);               // I2C compass object init
TinyGPSPlus gps;                                                                  // GPS init
HardwareSerial gpsSerial(1);                                                      // GPS UART init

// Servo definitions -------------------------------------------------------------
Servo leftServo;
Servo rightServo;

// Checkpoints -------------------------------------------------------------------
const double checkpoints[][2] = {
    {43.527208, -5.632747},
    {43.526658, -5.631508},
    {43.525135, -5.632324},
    {43.525510, -5.630115},
    {43.527104, -5.630602},
    {43.527208, -5.632747}
};                                                                                // Only the "j" dimension is limited as coordinates are always two-size. However, the number of checkpoints can be modified

const int totalCheckpoints = sizeof(checkpoints) / sizeof(checkpoints[0]);        // Autoadjastable checkpoint number the first 'sizeof' is the total amount of elements while the second is always 16 bytes, as it is the amount of elements to define a coordinate
int currentCheckpoint = 0;                                                        // Checkpoint controller

// Function prototypes -----------------------------------------------------------
void controlMotion(uint8_t leftWheel, uint8_t rightWheel, const String& motionMessage);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
float distanceTo(double lat1, double lon1, double lat2, double lon2);

// ===============================================================================
// SETUP
// ===============================================================================
void setup() {
  // Initialize serial monitor ---------------------------------------------------
  if(SERIAL_ENABLE){
    Serial.begin(115200);
  }

  // Initialize GPS --------------------------------------------------------------
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);           // Inicializacion de la comunicacion por serial para el GPS

  // Initialize compass ----------------------------------------------------------
  Wire.begin(SDA_EXT, SCL_EXT);
  if (!compass.begin()) {
    Serial.println("Compass not detected!");
    while (1);
  }

  // Initialize servos -----------------------------------------------------------
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  controlMotion(90, 90, "STOP MOTORS");

  Serial.println("Robot initialized. Starting navigation.");
}
// SETUP END =====================================================================

// ===============================================================================
// LOOP
// ===============================================================================
void loop() {
  // Read GPS data
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // If valid GPS data is available
  if (gps.location.isUpdated()) {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();
    double targetLat = checkpoints[currentCheckpoint][0];
    double targetLng = checkpoints[currentCheckpoint][1];

    Serial.print("Current Location: ");
    Serial.print(currentLat, 6);
    Serial.print(", ");
    Serial.println(currentLng, 6);

    // Calculate bearing to target
    double targetBearing = calculateBearing(currentLat, currentLng, targetLat, targetLng);

    // Get current heading from compass
    sensors_event_t event;                                                        // "event" data structure defined in the compass library
    compass.getEvent(&event);                                                     // Reads data in micro Teslas from the compass and stores it in 'event'
    double currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI; // Computation of the heading angle in degrees between positive x-axis and the point (x, y) -> arctan(x / y) * (180 / pi)
    if(currentHeading < 0) currentHeading += 360;                                 // When the angle is negative (arctan returns values between [-180, 180]), just convert it to its equivalent in the range [0, 360]

    Serial.print("Target Bearing: ");
    Serial.println(targetBearing);                                                // Next checkpoint orientation angle
    Serial.print("Current Heading: ");
    Serial.println(currentHeading);                                               // Current orientation angle

    // Adjust direction ----------------------------------------------------------
    double bearingDifference = targetBearing - currentHeading;                    // Bearing difference is the substraction of the target orientation and the current one
    if(bearingDifference > 180) bearingDifference -= 360;                         // Bearings must be in the [-180, 180] degree range, so if it exceeds 180º, it must be replaced by its equivalent negative counterpart in the valid range
    if(bearingDifference < -180) bearingDifference += 360;                        // Same for those bearings exceeding -180

    if(abs(bearingDifference) > 10){                                              // If the bearing difference is significant (10 degrees or more in any orientation)
      if(bearingDifference > 0){                                                  // If the bearing difference is positive, turn to the opposite direction, RIGHT
        controlMotion(60, 60, "TURNING RIGHT");
      }else{                                                                      // Do the same if the difference is negative
        controlMotion(120, 120, "TURNING LEFT");
      }
    }else{                                                                        // If not, no correction is needed, so move towards the next checkpoint
      controlMotion(60, 120, "MOVING FORWARD");
    }

    // Check if close to the target checkpoint -----------------------------------
    if(distanceTo(currentLat, currentLng, targetLat, targetLng) < 5.0){           // If the robot is 5 meters or less close to the checkpoint, it is considered to be there
      Serial.println("Checkpoint reached.");
      controlMotion(90, 90, "STOP MOTORS");                                       // The robot stops for a second
      delay(1000);
      currentCheckpoint++;                                                        // The next checkpoint is loaded
      if(currentCheckpoint >= totalCheckpoints){                                  // If it is the last checkpoint
        Serial.println("All checkpoints completed. Stopping robot.");             // Stop forever
        while(1) controlMotion(90, 90, "STOP MOTORS");
      }
    }
  }
}
// LOOP END ======================================================================

// -------------------------------------------------------------------------------
// MOTOR MOTION FUNCTION
// -------------------------------------------------------------------------------
void controlMotion(uint8_t leftWheel, uint8_t rightWheel, const String& motionMessage){  // | leftServo.write | rightServo.write |  EFFECT   |
  leftServo.write(leftWheel);                                                            // |       60        |       120        |  FORWARD  |
  rightServo.write(rightWheel);                                                          // |      120        |       120        |   LEFT    |
  Serial.println(motionMessage);                                                         // |       60        |        60        |   RIGHT   |
}                                                                                        // |       90        |        90        |   STOP    |
// MOTOR MOTION FUNCTION END -----------------------------------------------------

// -------------------------------------------------------------------------------
// BEARING COMPUTATION
// -------------------------------------------------------------------------------
double calculateBearing(double lat1, double lon1, double lat2, double lon2){     // Calculate straight-line bearing
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLon = lon2 - lon1;                                                      // As bearing is an angle, this function works with latitude and longitude directly as they are angles too
  double dLat = lat2 - lat1;
  
  double bearing = atan2(dLon, dLat) * 180 / PI;                                  // Convert radians to degrees
  if(bearing < 0) bearing += 360;                                                 // Normalize to 0-360 degrees
  return bearing;
}
// BEARING COMPUTATION END -------------------------------------------------------

// -------------------------------------------------------------------------------
// DISTANCE TO THE NEXT CHECKPOINT COMPUTATION
// -------------------------------------------------------------------------------
float distanceTo(double lat1, double lon1, double lat2, double lon2){             // Calculate Euclidean distance
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  const float METERS_PER_DEGREE_LON = METERS_PER_DEGREE_LAT * cos(lat1 * PI / 180);  // Adjust longitude distance based on latitude accounting for the Earth's curvature

  float dx = dLat * METERS_PER_DEGREE_LAT;                                        // Difference in latitude converted to meters
  float dy = dLon * METERS_PER_DEGREE_LON;                                        // Difference in longitude converted to meters

  return sqrt(dx * dx + dy * dy);                                                 // Pythagorean theorem
}
// DISTANCE TO THE NEXT CHECKPOINT COMPUTATION END -------------------------------
