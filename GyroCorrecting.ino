#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4IMU imu;
Zumo32U4LineSensors lineSensors;
#define NUM_SENSORS 5
//
//
// Setup for the gyro to measure degrees turned
//
//

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

//variable to store the data from the sensors
int lineSensorValues[NUM_SENSORS] = {0,0,500,0,0}; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  turnSensorSetup();
  delay(1000);
  turnSensorReset();
}


void loop() {
  // put your main code here, to run repeatedly:
  moveStraightForwardNoStop(100);
}



void moveStraightForwardNoStop(int fart) {

  //Set a variable for the angle
  int angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;

  
  while(true) {
    
       if (angle == 0) {
       //If gyro sensor is 0 degrees, run the same speed on both motors
        motors.setSpeeds(fart, fart);
       }
       //If the gyro is < 0 degrees turn slightly right
       else if (angle < 180) {
         motors.setSpeeds(fart + 50,fart);
       }  
      //else if gyro is > 0 degrees turn slightly left
       else if (angle > 180) {
       motors.setSpeeds(fart, fart +50);
     }
     readSensors();
     turnSensorUpdate();
     angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;
     lcd.clear();
     lcd.print(angle);
  }
}








//
//
//Functions involving the gyro and linesensors
//
//

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
    lcd.print((((uint32_t)turnAngle >> 16) * 360) >> 16);
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

void readSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}
