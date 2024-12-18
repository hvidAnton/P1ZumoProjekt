#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4OLED display;       // OLED display for feedback
Zumo32U4Buzzer buzzer;      // Buzzer for sound feedback
Zumo32U4ButtonA buttonA;    // Button A for user input
Zumo32U4ButtonC buttonC;    // Button C for user input
Zumo32U4Motors motors;      // Motor control
Zumo32U4Encoders encoders;  // Encoders for distance measurement (approx. 69.79 counts per cm)
Zumo32U4IMU imu;            // Inertial Measurement Unit for gyro readings

#define ENCODER_COUNTS_PER_CM 78;

// Define the number of points and coordinates arrays
const int numPoints = 16;
int16_t xCoords[numPoints] = { 0, 27, 32, 41, 51, 34, 22, 19, 45, 56, 72, 89, 76, 68, 12, 100 };
int16_t yCoords[numPoints] = { 0, 13, 15, 23, 42, 50, 57, 63, 84, 89, 96, 74, 29, 8, 30, 0 };

float gyroOffsetZ;       // Offset for gyro calibration
float currentAngle = 0;  // Initialize current angle
float lastAngle = 0;
long lastTime;  // Store last time in microseconds

// Variables for gyro
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

void setup() {
  Serial.begin(9600);         // Initialize serial communication
  display.print("Press A ");  // Display message on OLED and wait for button press
  buttonA.waitForButton();
  display.clear();
  display.print("go ");

  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  if (!imu.init()) {
    display.print("IMU Init Failed");
    while (1);  // stop the program if initialization fails
  }
  generateCoordinates(xCoords, yCoords, numPoints, 1); 
  shortDistensen();

  gyroCalibrate();  // Calibrate the gyro

  lastTime = micros();  // Initialize last time
  moveToPositions();    // Start moving to positions
  turnToAngle(90 - currentAngle);
  display.print("done");
}

void loop() {}

// Function to generate random coordinates with a minimum distance constraint
void generateCoordinates(int16_t xCoords[], int16_t yCoords[], int numPoints, float minDistance) {
  int seed = analogRead(A1);
  randomSeed(seed);
  Serial.println(seed);
  int count = 1;
  xCoords[0] = 0;
  yCoords[0] = 0;
  while (count < numPoints) {
    int16_t newX = random(0, 100);  // Random int16_t between 0 and 100
    int16_t newY = random(0, 100);
    bool valid = true;

    // Check the distance from all previously added points using calculateDistance
    for (int i = 0; i < count; i++) {
      if (calculateDistance(xCoords, yCoords, i, count) < minDistance) {
        valid = false;  // Too close to an existing point
        break;
      }
    }

    // If valid, add the point to the arrays
    if (valid) {
      xCoords[count] = newX;
      yCoords[count] = newY;
      count++;
    }
  }
  Serial.println("first coodes: ");
  printarray(xCoords, numPoints);
  printarray(yCoords, numPoints);
}


void shortDistensen() {
  for (int i = 0; i < numPoints - 2; i++) {
    float shortestDistance = 1e9;  // Use a large initial value for shortest distance
    int closestPointIndex = i;

    for (int j = i + 1; j < numPoints; j++) {
      float currentDistance = calculateDistance(xCoords, yCoords, i, j);
      if (currentDistance < shortestDistance) {
        shortestDistance = currentDistance;
        closestPointIndex = j;
      }
    }

    // Swap points if a closer point was found
    if (closestPointIndex != i + 1) {
      swapElements(yCoords, i + 1, closestPointIndex);
      swapElements(xCoords, i + 1, closestPointIndex);
    }

  }

  // Set the last point explicitly to (100, 0)
  xCoords[numPoints - 1] = 100;
  yCoords[numPoints - 1] = 0;

  Serial.println("Final coordinates:");
  printarray(xCoords, numPoints);
  printarray(yCoords, numPoints);
}


void gyroCalibrate() {
  gyroOffsetZ = 0;  // Reset the offset

  // Calibrate the gyro
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it
    while (!imu.gyroDataReady()) {
      delay(1);  // Small delay to prevent tight looping
    }
    imu.readGyro();

    // Add the Z axis reading to the total
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
}


void moveToPositions() {
  for (int i = 0; i < numPoints - 1; i++) {
    // Calculate angle and distance to the next point
    float angle = calculateAngle(xCoords, yCoords, i, i + 1);
    float dist = calculateDistance(xCoords, yCoords, i, i + 1);

    display.clear();
    display.print(xCoords[i + 1]);
    display.gotoXY(0, 1);
    display.print(yCoords[i + 1]);

    // Turn and move to the next point
    turnToAngle(angle);
    driveDistance(dist);
  }
  Serial.println("goal done");
  display.gotoXY(0, 0);
  display.print("done");
}

void turnToAngle(float targetAngle) {
  int turnSpeed = 100;
  long currentTime;
  float deltaTime;
  turnSensorReset();  // Reset turn sensor

  while (true) {
    currentTime = micros();                            // Get current time in microseconds
    deltaTime = (currentTime - lastTime) / 1000000.0;  // Calculate the time elapsed in seconds
    lastTime = currentTime;                            // Update last time
    currentAngle = getTurnAngleInDegrees();            // Update the current angle

    // Calculate the shortest path
    float angularDifference = targetAngle - currentAngle - lastAngle;
    if (angularDifference > 180) {  //calculates the shortest angular path to reach the target angle
      angularDifference -= 360;
    } else if (angularDifference < -180) {
      angularDifference += 360;
    }

    // If the angular difference is within 1 degrees, stop turning
    if (abs(angularDifference) <= 1) {
      break;
    }

    int direction = (angularDifference > 0) ? 1 : -1;  // If angularDifference is positive, direction is 1 (turn clockwise); if negative, direction is -1 (turn counterclockwise).
    motors.setSpeeds(-turnSpeed * direction, turnSpeed * direction);
  }
  lastAngle = targetAngle;
  // Debug output to monitor the angle updates
  motors.setSpeeds(0, 0);  // Stop the motors
  delay(1000);
}


float calculateAngle(int16_t xArray[], int16_t yArray[], int from, int to) {
  float deltaX = -xArray[to] - -xArray[from];
  float deltaY = yArray[to] - yArray[from];

  // Handle the case where deltaY is 0 (i.e., moving along the X axis)
  if (deltaY == 0) {
    if (deltaX > 0) {
      Serial.print("bruh");
      return 90;  // Move right (0 degrees)
    } else if (deltaX < 0) {
      return -90;  // Move left (180 degrees)
    }
  }

  // Normal calculation with atan2 if deltaY is not 0
  float angle = atan2(deltaX, deltaY) * (180 / PI);  // Convert to degrees

  // Normalize the angle to be within the range of 0 to 360 degrees
  if (angle < 0) {
    angle += 360;
  }

  return angle;
}

// This should be called to set the starting point for measuring a turn
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle. This should be called as frequently as possible while using the gyro to do turns
void turnSensorUpdate() {
  // Read the measurements from the gyro
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how much the robot has turned since the last update
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds. We need to convert those to the units of turnAngle, where 2^29 units represents 45 degrees
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // Convert angle in seconds to angle in degrees
  int32_t angle = ((turnAngle >> 16) * 360) >> 16;

  // Normalize the angle to be within the range of 0 to 360 degrees
  if (angle < 0) {
    angle += 360;
  }
  if (angle >= 360) {
    angle -= 360;
  }

  return angle;
}


void driveDistance(float distance) {
  int speed = 100;
  int counts = distance * ENCODER_COUNTS_PER_CM;  // Convert cm to encoder counts (adjust based on your measurements)
  encoders.getCountsAndResetLeft();               // Reset left encoder count
  encoders.getCountsAndResetRight();              // Reset right encoder count
  // Wait until the robot has traveled the desired distance
  while (encoders.getCountsLeft() < counts && encoders.getCountsRight() < counts) {
    if (getTurnAngleInDegrees() >= 1 + currentAngle) {  //If gyroscope angle is more than one degree off, the bot will start reducing the speed on one wheel.
      motors.setSpeeds(speed, speed - ((getTurnAngleInDegrees() - currentAngle) * 5)); // set speed to ajust for the angle 
    }

    if (getTurnAngleInDegrees() <= -1 + currentAngle) {  //If gyroscope angle is more than one degree off (in the other direction), the bot will start reducing the speed on the other wheel.
      motors.setSpeeds(speed + ((getTurnAngleInDegrees() - currentAngle)  * 5), speed); // set speed to ajust for the angle 
    }

    if (buttonC.getSingleDebouncedRelease()) {  //This button will break the loop.
      buttonC.waitForRelease();
      delay(100);
      break;
    }
  }

  motors.setSpeeds(0, 0);  // Stop the motors
  delay(100);
}

float calculateDistance(int16_t xArray[], int16_t yArray[], int from, int to) {
  return sqrt(pow(xArray[from] - xArray[to], 2) + pow(yArray[from] - yArray[to], 2));  // Calculate distance
}


void swapElements(int16_t array[], int x1, int x2) {
  array[x1] = array[x1] + array[x2];
  array[x2] = array[x1] - array[x2];
  array[x1] = array[x1] - array[x2];
}

float printarray(int16_t array[], int size) {
  for (int i = 0; i < size; i++) {
    Serial.print(array[i]);
    if (i < size - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();  // To ensure a new line after printing all elements
}
