#include<Wire.h>
#include<Zumo32U4.h>
#include <BasicLinearAlgebra.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;
Zumo32U4LCD lcd;
Zumo32U4LineSensors lineSensors;
#define NUM_SENSORS 5

int lineSensorValues[NUM_SENSORS];

bool useEmitters = true;

// White threshold; white returns values lower than this.
int threshold[NUM_SENSORS] = {300, 200, 160, 200, 230}; //{270,160,120,160,200};

struct LineSensorsWhite { // True if White, False if Black
  bool L;
  bool LC;
  bool C;
  bool RC;
  bool R;
};

LineSensorsWhite sensorsState = {0, 0, 0, 0, 0};



double countsL;
double countsR;

uint32_t turnAngle = 0;
int theta = 0;
double radTheta;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

using namespace BLA;

BLA::Matrix<2, 2> rotation;
BLA::Matrix<2, 2> rotation_inv;
BLA::Matrix<2> v;
BLA::Matrix<2> sumV;

void setup() {
  sumV.Fill(0);
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  //lineSensors.initFiveSensors(); // Initialize the 5 lineSensors.
  //calibrateThreshold();
  //buttonA.waitForPress();
  //buttonA.waitForRelease();
  //readSensors(sensorsState);
  turnSensorSetup();
  delay(500);
  turnSensorReset();  
  buttonA.waitForPress(); //Starts the program after a click on button A and a small delay
  delay(1000);
}

void loop() {
  moveForwardDistance(150,150,60);
  turn(100,90,'l');
  delay(50);
  moveForwardDistance(150,150,20);
  turn(100,90,'r');
  delay(50);
  moveForwardDistance(150,150,60);
  returnHome();
  buttonA.waitForPress();
}

void trackUpdate() {
  double countsL = encoders.getCountsAndResetLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
  double countsR = encoders.getCountsAndResetRight();
  double movement = ((countsL + countsR) / 900) * PI * 3.9;
  track(movement);

}
void moveForwardDistance(int spL, int spR, int centimeters) {
  double distance = 0;
  while (distance < centimeters) {
    motors.setSpeeds(spL, spR);
    double countsL = encoders.getCountsAndResetLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
    double countsR = encoders.getCountsAndResetRight();
    double movementUpdate = ((countsL + countsR) / 900) * PI * 3.9;
    distance += movementUpdate;
    track(movementUpdate);
  }
  motors.setSpeeds(0, 0);
  trackUpdate();
}

void moveForwardLine(int spL, int spR) {
  while (!sensorsState.C) {
    motors.setSpeeds(spL, spR);
    trackUpdate();
    readSensors(sensorsState);
  }
  motors.setSpeeds(0, 0);
  trackUpdate();
}

void moveForward(int spL, int spR) {
  motors.setSpeeds(spL, spR);
  trackUpdate();
}

void turn(int speed, int grader, char direction) {
  if (direction == 'l' || direction == 'L') {
    turnSensorReset();
    theta += grader;
    Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != grader) {
      Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
      motors.setSpeeds(-speed, speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
  } else if (direction == 'r' || direction == 'R') {
    turnSensorReset();
    theta -= grader;
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != 360 - grader) {
      motors.setSpeeds(speed, -speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
  }
}

void track(double distance) {
  //read the orientatation and encoders
  radTheta = theta * PI / 180;
  v(0) = distance;
  //construct the rotation matrix
  BLA::Matrix<2, 2> rotation = {cos(radTheta), sin(radTheta), -sin(radTheta), cos(radTheta)};

  //calculate the inverse matrix, in order to go from the local basis to the global
  BLA::Matrix<2, 2> rotation_inv = {cos(radTheta), -sin(radTheta), sin(radTheta), cos(radTheta)};
  //do the calculations
  // add the result onto the zum vector
  sumV += rotation_inv * v;

  //Serial << "rotation: " << rotation << '\n';
  //Serial.println("x-coordinate " + String(v(0)));
  //Serial.println("y-coordinate " + String(v(1)));
  //Serial.println("theta " + String(v(2)));
  //Serial << "sum " << sumV << '\n';
}

void returnHome() {
  //calculate the angle from the sumvector by tan(slope y / slope x)
  double angleSumV = 57.2957795 * atan2(sumV(1), sumV(0));
  lcd.clear();  
  lcd.print(sumV(0));
  lcd.gotoXY(0,1);
  lcd.print(sumV(1));
  buttonA.waitForPress();
  delay(1000);
  lcd.clear();
  lcd.print(angleSumV);
  turn(100,180-theta+angleSumV,'l');
  BLA::Matrix<2, 2> rotation = {cos(radTheta), sin(radTheta), -sin(radTheta), cos(radTheta)};
  BLA::Matrix<2> homeDistance = rotation * sumV;
  lcd.clear();
  lcd.print(homeDistance(0));
  
  moveForwardDistance(100,100,homeDistance(0));

}

void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  lcd.clear();
  lcd.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    lcd.gotoXY(0, 0);
    lcd.print((((uint32_t)turnAngle >> 16) * 360) >> 16);
    lcd.print(F("   "));
  }
  lcd.clear();
}

void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  int16_t m = micros();
  int16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;

}

void readSensors(LineSensorsWhite &state) {
  // Next line reads the sensor values and store them in the array lineSensorValues

  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

  // In the following lines use the values of the sensors to update the struct

  sensorsState.L = (lineSensorValues[0] < threshold[0]) ? 1 : 0;
  sensorsState.LC = (lineSensorValues[1] < threshold[1]) ? 1 : 0;
  sensorsState.C = (lineSensorValues[2] < threshold[2]) ? 1 : 0;
  sensorsState.RC = (lineSensorValues[3] < threshold[3]) ? 1 : 0;
  sensorsState.R = (lineSensorValues[4] < threshold[4]) ? 1 : 0;

}

void calibrateThreshold() {
  //local variabel til sorte og hvide værdier
  int black[NUM_SENSORS] = {0, 0, 0, 0, 0};
  int white[NUM_SENSORS] = {0, 0, 0, 0, 0};
  int belt[NUM_SENSORS] = {0, 0, 0, 0, 0};

  //print "Placer over 'sort' og tryk på button A"
  lcd.clear();
  lcd.print("Place");
  lcd.gotoXY(0, 1);
  lcd.print("black");

  // Vent på button A og aflæs sensorer
  buttonA.waitForPress();
  buttonA.waitForRelease();
  readSensors(sensorsState);

  // Gemme de sorte værdier
  for (int i = 0; i < 5; i++) {
    black[i] = lineSensorValues[i];
  }

  //print "placer over hvid og tryk på button A"
  lcd.clear();
  lcd.print("Place");
  lcd.gotoXY(0, 1);
  lcd.print("white");

  // vent på button a og aflæs sensorer
  buttonA.waitForPress();
  buttonA.waitForRelease();
  readSensors(sensorsState);


  // gem de hvide værdier
  for (int i = 0; i < 5; i++) {
    white[i] = lineSensorValues[i];
  }

  //lcd skal sige "placer ved start"
  lcd.clear();
  lcd.print("Place");
  lcd.gotoXY(0, 1);
  lcd.print("start");


  //læg hvid og sort sammen parvis, og divider med 2, brug derefter dette som threshold
  for (int i = 0; i < 5; i++) {
    threshold[i] = (black[i] + white[i]) / 2;
  }
  Serial.println(String(threshold[0]) + ", " + String(threshold[1]) + ", " + String(threshold[2]) + ", " + String(threshold[3]) + ", " + String(threshold[4]));

}
