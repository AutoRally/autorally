/**********************************************
   @file autorally_chassis.ino
   @author Brian Goldfain <bgoldfai@gmail.com>
   @date July 10, 2016
   @copyright 2016 Georgia Institute of Technology
   @brief Brains of the AutoRally chassis, for an Arduino Due

   @details Collects wheel rotation sensor, ESC, RC PWM, and error message data and sends it over the programming
            port to a compute box. Receives actuator commands over the programming port and controls the steering,
            throttle, and front brake using the Arduino Servo library

   @note install tc_lib found here: https://github.com/antodom/tc_lib to compile this program
   @note make sure to install the Due board into the arduino IDE otherwise the code will not compile
 ***********************************************/

#include <Servo.h>
#include "tc_lib.h"

static_assert(REFRESH_INTERVAL == 4500, "Please set the REFRESH_INTERVAL marco in Servo.h to be 4500");

//input declarations for the RC inputs
capture_tc6_declaration();
capture_tc7_declaration();
capture_tc8_declaration();

auto& cap_ovr = capture_tc6;
auto& cap_steer = capture_tc7; //pin 3, steer
auto& cap_thr = capture_tc8; //pin 11, throttle

// 50 ms max period, in us
#define SERVO_TIME_WINDOW 110000

#define MAX_MEASUREMENT_AGE 400  ///< Zero the reading at 400 ms

float analogVoltageScaler0; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs
float analogVoltageScaler1; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs

volatile unsigned long rightRearPeriod;      ///< Total period of wheel0
unsigned long rightRearPrevTimer;            ///< Previous timer value of wheel0
volatile unsigned long rightRearUpdateTime;  ///< Last update time for wheel0
volatile unsigned long leftRearPeriod;      ///< Total period of wheel0
unsigned long leftRearTimer;            ///< Previous timer value of wheel0
volatile unsigned long leftRearUpdateTime;  ///< Last update time for wheel0
volatile unsigned long rightFrontPeriod;      ///< Total period of wheel0
unsigned long rightFrontPrevTimer;            ///< Previous timer value of wheel0
volatile unsigned long rightFrontUpdateTime;  ///< Last update time for wheel0
volatile unsigned long leftFrontPeriod;      ///< Total period of wheel0
unsigned long leftFrontPrevtimer;            ///< Previous timer value of wheel0
volatile unsigned long leftFrontUpdatetime;  ///< Last update time for wheel0

int pulsesPerRevolution = 6;  ///< Number of magnets on each wheel
float divisor = 0.0; ///< Divisor calculated to turn absolute pulse   into rps
float rpsPublishPeriod = 10.0; ///< Period (in ms) for publishing arduinoData messages, 100hz
time_t rpsPublishTime; ///< Time that the last message was published

//pinout information, also avaialble in the Electronics Box Diagram
//int steerReadPin = 3;
//int throttleReadPin = 11;
//int manualModePin = 5;
int runStopPin = 6;

int frontBrakePin = 8;
int throttlePin = 9;
int steerPin = 10;

int buzzerPin = 7;

int rightRearRotationPin = 18;
int leftRearRotationPin = 19;
int rightFrontRotationPin = 20;
int leftFrontRotationPin = 21;

int steerSrvNeutralUs = 1500; ///< default neutral value for steering
int throttleSrvNeutralUs = 1500; ///< default neutral value for throttle
int frontBrakeSrvNeutralUs = 1950; ///< default neutral value for front brake (set this way since we have the reverse flag set to "True")
unsigned long timeOfLastServo = 0; ///< time that the last command message was received from the compute box

int buzzerState = 0; // state of the buzzer 0 is on off and 2+ is error
float buzzerDutyCycle = 0.10; //in %
int buzzerPeriod = 2000; // Period (in ms) for starting a buzz
int timeOfLastBuzz = 0; // time that the last buzz happened
int errorCount = 0;

///< have to receive actuator commants at at least 10Hz to control the platform (50-60Hz recommended)
unsigned long servoTimeoutMs = 100;

Servo steerSrv; ///< Arduino Servo object to control steering servo
Servo throttleSrv; ///< Arduino Servo object to control throttle
Servo frontBrakeSrv; ///< Arduino Servo object to control front brake servo

char castlLinkDeviceID = 0; ///< ESC Device ID (set to default)
int castleLinkPeriod = 200; ///< query ESC info at 5 Hz
char castleLinkRegisters[] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; ///< ESC data registers to query, details in Castle Serial Link
///< documentation
int castleLinkCurrentRegister = 0; // current register to query
char castleLinkData[2 * sizeof(castleLinkRegisters)]; ///< each register is 2 bytes
unsigned long timeOfCastleLinkData = 0; ///< last time the ESC was queried

char errorMsg[128] = ""; ///< error message periodically sent up to the compute box
/**
  @brief Sets up all parameters, attaches interrupts, and initializes Servo objects
*/
void setup()
{
  //Start with a null-terminated string
  errorMsg[0] = 0;
  //setup rotation sensor input pins
  pinMode(rightRearRotationPin, INPUT);
  pinMode(leftRearRotationPin, INPUT);
  pinMode(rightFrontRotationPin, INPUT);
  pinMode(leftFrontRotationPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(rightRearRotationPin, HIGH);
  digitalWrite(leftRearRotationPin, HIGH);
  digitalWrite(rightFrontRotationPin, HIGH);
  digitalWrite(leftFrontRotationPin, HIGH);
  digitalWrite(buzzerPin, LOW);

  //setup the runstop detect pin
  pinMode(runStopPin, INPUT);
  digitalWrite(runStopPin, HIGH);

  rpsPublishTime = 0;

  //attach interrupts for wheel rotation sensors
  attachInterrupt(rightRearRotationPin, int0, RISING);
  attachInterrupt(leftRearRotationPin, int1, RISING);
  attachInterrupt(rightFrontRotationPin, int2, RISING);
  attachInterrupt(leftFrontRotationPin, int3, RISING);

  rightRearPeriod = 0;      ///< Total period of wheel0
  rightRearPrevTimer = 0;            ///< Previous timer value of wheel0
  rightRearUpdateTime = 0;  ///< Last update time for wheel0
  leftRearPeriod = 0;      ///< Total period of wheel0
  leftRearTimer = 0;            ///< Previous timer value of wheel0
  leftRearUpdateTime = 0;  ///< Last update time for wheel0
  rightFrontPeriod = 0;      ///< Total period of wheel0
  rightFrontPrevTimer = 0;            ///< Previous timer value of wheel0
  rightFrontUpdateTime = 0;  ///< Last update time for wheel0
  leftFrontPeriod = 0;      ///< Total period of wheel0
  leftFrontPrevtimer = 0;            ///< Previous timer value of wheel0
  leftFrontUpdatetime = 0;  ///< Last update time for wheel0

  //attach servos
  steerSrv.attach(steerPin);
  throttleSrv.attach(throttlePin);
  frontBrakeSrv.attach(frontBrakePin);

  //default servos to centered
  steerSrv.writeMicroseconds(steerSrvNeutralUs);
  throttleSrv.writeMicroseconds(throttleSrvNeutralUs);
  frontBrakeSrv.writeMicroseconds(frontBrakeSrvNeutralUs);

  Serial.begin(115200);
  Serial3.begin(115200);
  //lower timeout so read() returns faster if no data
  Serial3.setTimeout(10);

  //clear serial link command buffer
  for (int i = 0; i < 5; i++)
  {
    Serial3.write(static_cast<byte>(0x00));
//    Serial3.write('0x00');

  }

  // Setup input captures for RC
  cap_ovr.config(SERVO_TIME_WINDOW);
  cap_steer.config(SERVO_TIME_WINDOW);
  cap_thr.config(SERVO_TIME_WINDOW);

}

/**
  @brief Main body of program, gathers and publishes sensor data.
*/
void loop()
{
  //if enough data is available (a command msg is 9 bytes)
  while (Serial.available() >= 9)
  {
    //make sure we are framed at the beginning of a message
    if (Serial.read() == '#')
    {
      char msgType = Serial.read();
      //errorMsg += "received message type " + String(msgType);
      //unpack servo message and set the servos if at least 7 bytes available (6 payload + \n)
      if (msgType == 's')
      {
        char buf[7];
        if (Serial.readBytes(buf, 7) == 7)
        {
          short steer = ((short)buf[0] << 8) + buf[1] & 0xFFFF;
          short throttle = ((short)buf[2] << 8) + buf[3] & 0xFFFF;
          short frontBrake = ((short)buf[4] << 8) + buf[5] & 0xFFFF;

          timeOfLastServo = millis();

          //command actuators
          steerSrv.writeMicroseconds(steer);
          throttleSrv.writeMicroseconds(throttle);
          frontBrakeSrv.writeMicroseconds(frontBrake);
        }
      }
    }
  }

  if(buzzerState >= 2) {
    if(timeOfLastBuzz + ((buzzerDutyCycle * 2 * buzzerPeriod) / 6) > millis()) {
      digitalWrite(buzzerPin, HIGH);
    } else if(timeOfLastBuzz + (buzzerPeriod / 6) > millis()) {
      digitalWrite(buzzerPin, LOW);
    } else if(buzzerState >= 4) {
      digitalWrite(buzzerPin, LOW);
      if(timeOfLastBuzz + buzzerPeriod < millis()) {
        buzzerState = 0;
        timeOfLastBuzz = millis();
      }
    } else {
      buzzerState++;
      timeOfLastBuzz = millis();
    }
  } else if(!digitalRead(runStopPin)) {
    if(timeOfLastBuzz + buzzerDutyCycle * buzzerPeriod > millis()) {
      digitalWrite(buzzerPin, HIGH);
    } else if(timeOfLastBuzz + buzzerPeriod > millis()) {
      digitalWrite(buzzerPin, LOW);
    } else {
      timeOfLastBuzz = millis();
    }
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  //if no servo msg has been received in a while, set them to neutral
  if (timeOfLastServo + servoTimeoutMs < millis())
  {
    steerSrv.writeMicroseconds(steerSrvNeutralUs);
    throttleSrv.writeMicroseconds(throttleSrvNeutralUs);
    frontBrakeSrv.writeMicroseconds(frontBrakeSrvNeutralUs);
  }

  //send wheel speeds and RC input back to compute box
  if (rpsPublishTime + rpsPublishPeriod < millis())
  {
    rpsPublishTime = millis();

    float leftFront, rightFront, leftRear, rightRear;
    //getrps(leftFront, rightFront, leftBack, rightBack);
    getrps(rightRear, leftRear, rightFront, leftFront);

    Serial.print("#w");
    Serial.print(leftFront, 3);
    Serial.print(",");
    Serial.print(rightFront, 3);
    Serial.print(",");
    Serial.print(leftRear, 3);
    Serial.print(",");
    Serial.print(rightRear, 3);
    Serial.print('\n');

    uint32_t rc_steer, rc_throttle, rc_frontBrake;
    getRcWidths(rc_steer, rc_throttle, rc_frontBrake);

    Serial.print("#r");
    Serial.print(rc_steer);
    Serial.print(",");
    Serial.print(rc_throttle);
    Serial.print(",");
    Serial.print(rc_frontBrake);
    Serial.print(",");
    Serial.print(digitalRead(runStopPin));
    Serial.print('\n');
  }

  //query ESC data and send it to the compute box
  if (timeOfCastleLinkData + castleLinkPeriod < millis())
  {
    if(castleLinkCurrentRegister >= sizeof(castleLinkRegisters)/sizeof(char)) {
       castleLinkCurrentRegister = 0;
       timeOfCastleLinkData = millis();
       Serial.print("#c");
       Serial.write(castleLinkData, sizeof(castleLinkData));
       Serial.print('\n');
    } else {
      getCastleSerialLinkData();
    }
  }

  //send any error text up to the compute box, the message may contain multiple, concatenated errors
  if (errorMsg[0] != 0)
  {
    if(errorCount >= 3) {
      buzzerState =  buzzerState >= 2 ? buzzerState : 2;
    }
    Serial.print("#e");
    Serial.print(errorMsg);
    Serial.print('\n');
    errorMsg[0] = 0;
  }
}

/**
  @brief get timing information for the RC input channels steering, throttle, autonomousEnabled
  @param[out] rc_1 RC steering pulse width in us
  @param[out] rc_2 RC throttle pulse width in us
  @param[out] rc_3 RC front brake pulse width in us
*/
void getRcWidths(uint32_t &rc_steer, uint32_t &rc_throttle, uint32_t &rc_frontBrake)
{
  uint32_t ret, duty, period, pulses;
  ret = cap_steer.get_duty_period_and_pulses(duty, period, pulses);
  rc_steer = (double)duty / (double)cap_steer.ticks_per_usec();

  ret = cap_thr.get_duty_period_and_pulses(duty, period, pulses);
  rc_throttle = (double)duty / (double)cap_thr.ticks_per_usec();

  ret = cap_ovr.get_duty_period_and_pulses(duty, period, pulses);
  rc_frontBrake = (double)duty / (double)cap_ovr.ticks_per_usec();
}

/**
  @brief compute wheel speeds based on interrupt counts
  @param[out] rightRear left front wheel speed in m/s
  @param[out] leftRear right front wheel speed in m/s
  @param[out] rightFront left wheel speed in m/s
  @param[out] leftFront right rear wheel speed in m/s
*/
void getrps(float &rightRear, float &leftRear, float &rightFront, float &leftFront)
{
  unsigned long rightRearTemp, leftRearTemp, rightFrontTemp, leftFrontTemp;
  unsigned long rightRearT, leftRearT, rightFrontT, leftFrontT;

  noInterrupts();
  rightRearT = rightRearUpdateTime;
  rightRearTemp = rightRearPeriod;
  leftRearT = leftRearUpdateTime;
  leftRearTemp = leftRearPeriod;
  rightFrontT = rightFrontUpdateTime;
  rightFrontTemp = rightFrontPeriod;
  leftFrontT = leftFrontUpdatetime;
  leftFrontTemp = leftFrontPeriod;
  interrupts();

  //  Serial.print("DebugVals: ");
  //  Serial.print(rightRearT);
  //  Serial.print(",");
  //  Serial.print(rightRearTemp);
  //  Serial.print("\n");
  unsigned long t = millis();

  if ((rightRearT + MAX_MEASUREMENT_AGE) < t) {
    rightRear = 0;
  } else {
    rightRear = 1000000.0 / (pulsesPerRevolution * rightRearTemp);
  }

  if ((leftRearT + MAX_MEASUREMENT_AGE) < t) {
    leftRear = 0;
  } else {
    leftRear = 1000000.0 / (pulsesPerRevolution * leftRearTemp);
  }

  if ((rightFrontT + MAX_MEASUREMENT_AGE) < t) {
    rightFront = 0;
  } else {
    rightFront = 1000000.0 / (pulsesPerRevolution * rightFrontTemp);
  }

  if ((leftFrontT + MAX_MEASUREMENT_AGE) < t) {
    leftFront = 0;
  } else {
    leftFront = 1000000.0 / (pulsesPerRevolution * leftFrontTemp);
  }

}

/**
  @brief Interrupt service routine 0 (right rear rpm sensor)
*/
void int0()
{
  unsigned long t = micros();
  rightRearPeriod = t - rightRearPrevTimer;
  rightRearPrevTimer = t;
  rightRearUpdateTime = millis();
}

/**
  @brief Interrupt service routine 1 (left rear rpm sensor)
*/
void int1()
{
  unsigned long t = micros();
  leftRearPeriod = t - leftRearTimer;
  leftRearTimer = t;
  leftRearUpdateTime = millis();
}

/**
  @brief Interrupt service routine 2 (right front rpm sensor)
*/
void int2()
{
  unsigned long t = micros();
  rightFrontPeriod = t - rightFrontPrevTimer;
  rightFrontPrevTimer = t;
  rightFrontUpdateTime = millis();
}

/**
  @brief Interrupt service routine 3 (left front rpm sensor)
*/
void int3()
{
  unsigned long t = micros();
  leftFrontPeriod = t - leftFrontPrevtimer;
  leftFrontPrevtimer = t;
  leftFrontUpdatetime = millis();
}

/**
  @brief Send and receive each desired ESC state register, put values into array to be sent to compute box
  @return Status of data parsing. If anything fails, returns false immediately

  @note received ESC data is packed into castleLinkData[]
*/
bool getCastleSerialLinkData()
{
  char request[5];
  char response[3];

  request[0] = (0x1 << 7) + castlLinkDeviceID;
  request[2] = 0;
  request[3] = 0;

  request[1] = castleLinkRegisters[castleLinkCurrentRegister];
  request[4] = castleChecksum(request);

  Serial3.write(request, 5);

  //read 3 byte responses
  int bytesRead = Serial3.readBytes(response, 3);
  if (bytesRead == 3)
  {

    if ( (char)(response[0] + response[1] + response[2]) == 0)
    {
      if (response[0] == 255 && response[1] == 255) // error from serial link indicated by 0xFFFF
      {
        sprintf(errorMsg,"castle link comm error on register %X",response[0]);
        //errorMsg += "castle link comm error on register " + ('0' + response[0]) + ',';
        //invalid register of corrupted command
      } else
      {
        castleLinkData[2 * castleLinkCurrentRegister] = response[0];
        castleLinkData[2 * castleLinkCurrentRegister + 1] = response[1];
      }
    } else
    {
      sprintf(errorMsg,"castle link comm failed checksum");
      //errorMsg += "castle link comm failed checksum,";
      castleLinkCurrentRegister = 0;
      errorCount++;
      return false;
    }
  } else
  {
    sprintf(errorMsg,"wrong number of bytes read from castle link: %d for register %d",bytesRead,castleLinkCurrentRegister);
    //errorMsg += "wrong number of bytes read from castle link: " + String(bytesRead) + " for register " + String(castleLinkCurrentRegister) + ",";
    castleLinkCurrentRegister = 0;
    errorCount++;
    return false;
  }
  castleLinkCurrentRegister++;
  errorCount = 0;
  return true;
}

/**
  @brief compute checksum according to Castle Serial Link documentation
  @param msg 4 byte message to compute checksum over
  @return checksum 1 byte
*/
char castleChecksum(char* msg)
{
  return (0 - (msg[0] + msg[1] + msg[2] + msg[3]));
}
