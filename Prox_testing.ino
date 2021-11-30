#include <Wire.h>
#include <Zumo32U4.h>
#define numberOfBrightnessLevels 10

Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;

int brightnessLevels[numberOfBrightnessLevels];

int proxStatus = 0; //Variable for determining what the proximity is detecting. 0 is nothing, 1 is obstacle, 2 is wad

bool sustainLoop = true;

//
//Gyro variables
//

uint32_t turnAngle = 0;

int16_t turnRate;

int16_t gyroOffset;

uint16_t gyroLastUpdate = 0;

void setup() {
  Serial.begin(9600);
  proxSensors.initFrontSensor();
  for (int i = 0; i < numberOfBrightnessLevels; i++) {
    brightnessLevels[i] = i;
    Serial.println("Brightnesslevel(" + String(i) + ") = " + String(brightnessLevels[i]));
  }
  proxSensors.setBrightnessLevels(brightnessLevels, numberOfBrightnessLevels);
  turnSensorSetup();
  delay(1000);
  turnSensorReset();
}

void loop() {
  while(sustainLoop){
  proxStatus = 0;
  motors.setSpeeds(100, 100);
  proxRead();
  switch (proxStatus) {

    case 0:
      break;

    case 1:
      ledGreen(true);
      delay(4000);
      ledGreen(false);
      break;

    case 2:
      ledRed(true);
      delay(4000);
      ledRed(false);
      break;

  }
  if(proxStatus != 0){
    sustainLoop = false;
  }
}
}

void proxRead() {
  proxSensors.read();
  int proximityLeft = proxSensors.countsFrontWithLeftLeds();
  int proximityRight = proxSensors.countsFrontWithRightLeds();
  Serial.println("proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight));
  if (proximityRight == 9 || proximityLeft == 9) { //If something triggers this number, there is either an obstacle or wad in front of the robot
    motors.setSpeeds(0, 0);
    moveForward(100, 10); //move forward to check the number again,
    proxSensors.read();
    proximityLeft = proxSensors.countsFrontWithLeftLeds();
    proximityRight = proxSensors.countsFrontWithRightLeds();
    if (proximityRight <= 8 || proximityLeft <= 8) {
      if(proximityLeft < proximityRight){
        turn(100, 35, 'r');
        proxSensors.read();
        proximityLeft = proxSensors.countsFrontWithLeftLeds();
        proximityRight = proxSensors.countsFrontWithRightLeds();
        if(proximityRight > 8){
          proxStatus = 1;
        }else{
          proxStatus = 2; //Define the status of the proximity readings as detecting a wad.
        }
        turn(100, 35, 'l');
      }else if(proximityLeft > proximityRight){
        turn(100, 35, 'l');
        proxSensors.read();
        proximityLeft = proxSensors.countsFrontWithLeftLeds();
        proximityRight = proxSensors.countsFrontWithRightLeds();
        if(proximityLeft > 8){
          proxStatus = 1;
        }else{
          proxStatus = 2; //Define the status of the proximity readings as detecting a wad.
        }
        turn(100, 35, 'r');
      }
    }
    else {
      proxStatus = 1; //Define the status of the proximity readings as detecting an obstacle.
    }
  }
  lcd.clear();
  lcd.print("L:" + String(proximityLeft) + "R:" + String(proximityRight));
}


void moveForward(int fart, double distance) {
  double globalMovement = 0; //initierer variabler med globalMovement og counts
  double counts = encoders.getCountsAndResetLeft(); //Resetter venstre encoder og sætter counts til det førhenværende antal counts
  counts = encoders.getCountsLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
  globalMovement = (counts / 909.7) * PI * 3.9;
  motors.setSpeeds(fart, fart);
  while (globalMovement < distance) {
    counts = encoders.getCountsLeft();
    globalMovement = (counts / 909.7) * PI * 3.9;
  }
  motors.setSpeeds(0, 0);
  globalMovement = 0;
  counts = encoders.getCountsAndResetLeft();
}

//Function for turning with a set speed, degrees and direction of turn.
void turn(int fart, int grader, char direction) {
  if (direction == 'l' || direction == 'L') {
       turnSensorReset();
       while (((((int32_t)turnAngle >> 16) * 360) >> 16) < grader) {
         motors.setSpeeds(-fart,fart);
         turnSensorUpdate();
      }
      motors.setSpeeds(0,0);
  } else if (direction == 'r' || direction == 'R') {
       turnSensorReset();
       while (((((int32_t)turnAngle >> 16) * 360) >> 16) > -grader) {
          motors.setSpeeds(fart,-fart);
          turnSensorUpdate();
       }
       motors.setSpeeds(0,0);
  } 
}

void turnSensorSetup()
{
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
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
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
    lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    lcd.print(F("   "));
  }
  lcd.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
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
