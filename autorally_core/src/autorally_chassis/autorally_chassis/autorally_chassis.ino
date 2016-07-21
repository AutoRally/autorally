

/**********************************************
 * @file autorally_chassis.ino
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 10, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Brains of the AutoRally chassis
 *
 * @details Collects wheel rotation sensor, ESC, RC PWM, and error message data and sends it over the programming
 *          port to a compute box. Receives actuator commands over the programming port and controls the steering,
 *          throttle, and front brake using the Arduino Servo library
 ***********************************************/

#include <Servo.h> 
#include "tc_lib.h"

//input declarations for the RC inputs
capture_tc6_declaration();
capture_tc7_declaration();
capture_tc8_declaration();

auto& cap_ovr=capture_tc6;
auto& cap_steer=capture_tc7;
auto& cap_thr=capture_tc8;

// 50 ms max period, in us
#define SERVO_TIME_WINDOW 50000

#define MAX_MEASUREMENT_AGE 400  ///< Zero the reading at 400 ms

float analogVoltageScaler0; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs
float analogVoltageScaler1; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs

volatile unsigned long w0period;      ///< Total period of wheel0
unsigned long w0prevtimer;            ///< Previous timer value of wheel0
volatile unsigned long w0updatetime;  ///< Last update time for wheel0
volatile unsigned long w1period;      ///< Total period of wheel0
unsigned long w1prevtimer;            ///< Previous timer value of wheel0
volatile unsigned long w1updatetime;  ///< Last update time for wheel0
volatile unsigned long w2period;      ///< Total period of wheel0
unsigned long w2prevtimer;            ///< Previous timer value of wheel0
volatile unsigned long w2updatetime;  ///< Last update time for wheel0
volatile unsigned long w3period;      ///< Total period of wheel0
unsigned long w3prevtimer;            ///< Previous timer value of wheel0
volatile unsigned long w3updatetime;  ///< Last update time for wheel0

//volatile unsigned int rc_risingEdge4; ///< Record the rising edge of rc signal on IC4
//volatile unsigned int rc_width4;      ///< Record the pulse width of the rc signal on IC4
//volatile unsigned int rc_risingEdge5; ///< Record the rising edge time of rc signal on IC5
//volatile unsigned int rc_width5;      ///< Record the pulse width of the rc signal on IC5
//
//volatile unsigned int rc_width6 = 0;  ///< Record the pulse width of the rc signal on IC5

int pulsesPerRevolution = 6;  ///< Number of magnets on each wheel
float divisor = 0.0; ///< Divisor calculated to turn absolute pulse count into rps
float rpsPublishPeriod = 13.0; ///< Period (in ms) for publishing arduinoData messages, 70hz
time_t rpsPublishTime; ///< Time that the last message was published

//pinout information, also avaialble in the Electronics Box Diagram
int steerReadPin = 3;
int throttleReadPin = 4;
int manualModePin = 5;
int runStopPin = 6;

int frontBrakePin = 8;
int throttlePin = 9;
int steerPin = 10;

int rightRearRotationPin = 18;
int leftRearRotationPin = 19;
int rightFrontRotationPin = 20;
int leftFrontRotationPin = 21;

int steerSrvNeutralUs = 1500; ///< default neutral value for steering
int throttleSrvNeutralUs = 1500; ///< default neutral value for throttle
int frontBrakeSrvNeutralUs = 1500; ///< default neutral value for front brake
unsigned long timeOfLastServo = 0; ///< time that the last command message was received from the compute box

//have to receive actuator commants at at least 10Hz to control the platform (50-60Hz recommended)
unsigned long servoTimeoutMs = 100;

Servo steerSrv; ///< Arduino Servo object to control steering servo
Servo throttleSrv; ///< Arduino Servo object to control throttle
Servo frontBrakeSrv; ///< Arduino Servo object to control front brake servo

char castlLinkDeviceID = 0; ///< ESC Device ID (set to default)
int castleLinkPeriod = 200; ///< query ESC info at 5 Hz
char castleLinkRegisters[] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; ///< ESC data registers to query, details in Castle Serial Link
                                                          ///< documentation
char castleLinkData[2*sizeof(castleLinkRegisters)]; ///< each register is 2 bytes
unsigned long timeOfCastleLinkData = 0; ///< last time the ESC was queried

String errorMsg = ""; ///< error message periodically sent up to the compute box
/**
* @brief Sets up all parameters, attaches interrupts, and initializes Servo objects
*/
void setup()
{
  
  //setup rotation sensor input pins
  pinMode(rightRearRotationPin, INPUT);
  pinMode(leftRearRotationPin, INPUT);
  pinMode(rightFrontRotationPin, INPUT);
  pinMode(leftFrontRotationPin, INPUT);
  digitalWrite(rightRearRotationPin,HIGH);
  digitalWrite(leftRearRotationPin,HIGH);
  digitalWrite(rightFrontRotationPin,HIGH);
  digitalWrite(leftFrontRotationPin,HIGH);

  //setup the runstop detect pin
  pinMode(runStopPin, INPUT);
  digitalWrite(runStopPin, HIGH);
  
  rpsPublishTime = 0;
  
  //attach interrupts for wheel rotation sensors
  attachInterrupt(rightRearRotationPin, int0, RISING);
  attachInterrupt(leftRearRotationPin, int1, RISING);
  attachInterrupt(rightFrontRotationPin, int2, RISING);
  attachInterrupt(leftFrontRotationPin, int3, RISING);
  
  w0period = 0;      ///< Total period of wheel0
  w0prevtimer = 0;            ///< Previous timer value of wheel0
  w0updatetime = 0;  ///< Last update time for wheel0
  w1period = 0;      ///< Total period of wheel0
  w1prevtimer = 0;            ///< Previous timer value of wheel0
  w1updatetime = 0;  ///< Last update time for wheel0
  w2period = 0;      ///< Total period of wheel0
  w2prevtimer = 0;            ///< Previous timer value of wheel0
  w2updatetime = 0;  ///< Last update time for wheel0
  w3period = 0;      ///< Total period of wheel0
  w3prevtimer = 0;            ///< Previous timer value of wheel0
  w3updatetime = 0;  ///< Last update time for wheel0

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
  for(int i = 0; i < 5; i++)
  {
    Serial3.write('0x00');
  }

  // Setup input captures for RC
  cap_ovr.config(SERVO_TIME_WINDOW);
  cap_steer.config(SERVO_TIME_WINDOW);
  cap_thr.config(SERVO_TIME_WINDOW);

}

/**
* @brief Main body of program, gathers and publishes sensor data.
*/
void loop()
{
  //if enough data is available (a command msg is 9 bytes)
  if(Serial.available() >= 9)
  {
    //make sure we are framed at the beginning of a message
    if(Serial.read() == '#')
    {
      char msgType = Serial.read();
      //errorMsg += "received message type " + String(msgType);
      //unpack servo message and set the servos if at least 7 bytes available (6 payload + \n)
      if(msgType == 's')
      {
        char buf[7];
        if(Serial.readBytes(buf, 7) == 7)
        {
          short steer = ((short)buf[0]<<8) + buf[1]&0xFFFF;
          short throttle = ((short)buf[2]<<8) + buf[3]&0xFFFF;
          short frontBrake = ((short)buf[4]<<8) + buf[5]&0xFFFF;
          
          timeOfLastServo = millis();
          
          //command actuators
          steerSrv.writeMicroseconds(steer);
          throttleSrv.writeMicroseconds(throttle);
          frontBrakeSrv.writeMicroseconds(frontBrake);
        }
      }   
    }
  }

  //if no servo msg has been received in a while, set them to neutral
  if(timeOfLastServo+servoTimeoutMs < millis())
  {
    steerSrv.writeMicroseconds(steerSrvNeutralUs);
    throttleSrv.writeMicroseconds(throttleSrvNeutralUs);
    frontBrakeSrv.writeMicroseconds(frontBrakeSrvNeutralUs);
  }

  //send wheel speeds and RC input back to compute box
  if(rpsPublishTime+rpsPublishPeriod < millis())
  {
    rpsPublishTime = millis();
    
    float leftFront, rightFront, leftBack, rightBack;  
    getrps(leftFront, rightFront, leftBack, rightBack);

    Serial.print("#w");
    Serial.print(leftFront,3);
    Serial.print(",");
    Serial.print(rightFront,3);
    Serial.print(",");
    Serial.print(leftBack,3);
    Serial.print(",");
    Serial.print(rightBack,3);
    Serial.print('\n');

    uint32_t rc_1, rc_2, rc_3;
    getRcWidths(rc_1, rc_2, rc_3);

    Serial.print("#r");
    Serial.print(rc_1);
    Serial.print(",");
    Serial.print(rc_2);
    Serial.print(",");
    Serial.print(rc_3);
    Serial.print(",");
    Serial.print(digitalRead(runStopPin));
    Serial.print('\n');
  }

  //query ESC data and send it to the compute box
  if(timeOfCastleLinkData+castleLinkPeriod < millis())
  {
    timeOfCastleLinkData = millis();
    if(getCastleSerialLinkData())
    {
      Serial.print("#c");
      Serial.write(castleLinkData, sizeof(castleLinkData));
      Serial.print('\n');
    }
  }

  //send any error text up to the compute box, the message may contain multiple, concatenated errors
  if(errorMsg.length())
  {
    Serial.print("#e");
    Serial.print(errorMsg);
    Serial.print('\n');
    errorMsg = "";
  }
}

/**
* @brief get timing information for the RC input channels steering, throttle, autonomousEnabled
* @param[out] rc_1 RC steering pulse width in us
* @param[out] rc_2 RC throttle pulse width in us
* @param[out] rc_3 RC front brake pulse width in us
*/
void getRcWidths(uint32_t &rc_1, uint32_t &rc_2, uint32_t &rc_3)
{
  uint32_t ret, duty, period, pulses;
  ret = cap_steer.get_duty_period_and_pulses(duty, period, pulses);
  rc_1 = (double)duty/(double)cap_steer.ticks_per_usec();
  ret = cap_thr.get_duty_period_and_pulses(duty, period, pulses);
  rc_2 = (double)duty/(double)cap_thr.ticks_per_usec();
  ret = cap_ovr.get_duty_period_and_pulses(duty, period, pulses);
  rc_3 = (double)duty/(double)cap_ovr.ticks_per_usec();
}

/**
* @brief compute wheel speeds based on interrupt counts
* @param[out] w0 left front wheel speed in m/s
* @param[out] w1 right front wheel speed in m/s
* @param[out] w2 left wheel speed in m/s
* @param[out] w3 right rear wheel speed in m/s
*/
void getrps(float &w0, float &w1, float &w2, float &w3)
{
  unsigned long w0temp, w1temp, w2temp, w3temp;
  unsigned long w0t, w1t, w2t, w3t;

  noInterrupts();
  w0t = w0updatetime;
  w0temp = w0period;
  w1t = w1updatetime;
  w1temp = w1period;
  w2t = w2updatetime;
  w2temp = w2period;
  w3t = w3updatetime;
  w3temp = w3period;
  interrupts();

//  Serial.print("DebugVals: ");
//  Serial.print(w0t);
//  Serial.print(",");
//  Serial.print(w0temp);
//  Serial.print("\n");
  unsigned long t = millis();
  
  if ((w0t + MAX_MEASUREMENT_AGE) < t) {
    w0 = 0;
  } else {
    w0 = 1000000.0 / (pulsesPerRevolution * w0temp);
  }
  
  if ((w1t + MAX_MEASUREMENT_AGE) < t) {
    w1 = 0;
  } else {
    w1 = 1000000.0 / (pulsesPerRevolution * w1temp);
  }
  
  if ((w2t + MAX_MEASUREMENT_AGE) < t) {
    w2 = 0;
  } else {
    w2 = 1000000.0 / (pulsesPerRevolution * w2temp);
  }
  
  if ((w3t + MAX_MEASUREMENT_AGE) < t) {
    w3 = 0;
  } else {
    w3 = 1000000.0 / (pulsesPerRevolution * w3temp);
  }
  
}

/**
* @brief Interrupt service routine 0 (right rear rpm sensor)
*/
void int0()
{
  unsigned long t = micros();
  w0period = t - w0prevtimer;
  w0prevtimer = t;
  w0updatetime = millis();
}

/**
* @brief Interrupt service routine 1 (left rear rpm sensor)
*/
void int1()
{
  unsigned long t = micros();
  w1period = t - w1prevtimer;
  w1prevtimer = t;
  w1updatetime = millis();
}

/**
* @brief Interrupt service routine 2 (right front rpm sensor)
*/
void int2()
{
  unsigned long t = micros();
  w2period = t - w2prevtimer;
  w2prevtimer = t;
  w2updatetime = millis();
}

/**
* @brief Interrupt service routine 3 (left front rpm sensor)
*/
void int3()
{
  unsigned long t = micros();
  w3period = t - w3prevtimer;
  w3prevtimer = t;
  w3updatetime = millis();
}

/**
* @brief Send and receive each desired ESC state register, put values into array to be sent to compute box
* @return Status of data parsing. If anything fails, returns false immediately
*
* @note received ESC data is packed into castleLinkData[]
*/
bool getCastleSerialLinkData()
{
  char request[5];
  char bufRec[3];
  int count = 0;

  request[0] = (0x1<<7) + castlLinkDeviceID;
  request[2] = 0;
  request[3] = 0;
  
  for(int i = 0; i < sizeof(castleLinkRegisters); i++)
  {
    request[1] = castleLinkRegisters[i];  
    request[4] = castleChecksum(request);

    Serial3.write(request, 5);

    //read 3 byte responses
    int bytesRead = Serial3.readBytes(bufRec, 3);
    if(bytesRead == 3)
    {
      
      if( (char)(bufRec[0] + bufRec[1] + bufRec[2]) == 0)
      {
        if(bufRec[0] == 255 && bufRec[1] == 255) // error from serial link indicated by 0xFFFF
        {
          errorMsg += "castle link comm error on register " + ('0'+bufRec[0]) + ',';
          //invalid register of corrupted command
        } else
        {
          castleLinkData[2*count] = bufRec[0];
          castleLinkData[2*count+1] = bufRec[1];
          ++count;
        }
      } else
      {
        errorMsg += "castle link comm failed checksum,";
        return false;
      }
    } else
    {
      errorMsg += "castle link comm wrong number of bytes read " + String(bytesRead) + ",";
      return false;
    }
  }
  return true;
}

/**
* @brief compute checksum according to Castle Serial Link documentation
* @param msg 4 byte message to compute checksum over
* @return checksum 1 byte
*/
char castleChecksum(char* msg)
{
  return (0 - (msg[0] + msg[1] + msg[2] + msg[3]));
}