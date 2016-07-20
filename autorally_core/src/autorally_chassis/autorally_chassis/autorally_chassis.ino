

/**********************************************
 * @file arduinoDataSerial,ino
 * @author Alex Bettadapur <alexbettadapur@gmail.com>
 * @date January 22, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief Communicates with sensors connected to Arduino
 *
 * @details Same function as the arduinoData.ino file, except modified
 *          to transfer data over serial. This program
 *          is meant to run on an Arduino Mega. It collects data
 *          from 4 Hall Effect sensors to measure wheel rotation
 *          and the voltage of the onboard data packs and publishes
 *          all data in one arduinoData message.
 *          This program also handles triggering the cameras.
 ***********************************************/

//#include <avr/io.h>
//#include <avr/interrupt.h>
#include <Servo.h> 
#include "tc_lib.h"

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

//volatile unsigned int rc_risingEdge4;  ///< Record the rising edge of rc signal on IC4
//volatile unsigned int rc_width4;       ///< Record the pulse width of the rc signal on IC4
//volatile unsigned int rc_risingEdge5;  ///< Record the rising edge time of rc signal on IC5
//volatile unsigned int rc_width5;       ///< Record the pulse width of the rc signal on IC5
//
//volatile unsigned int rc_width6 = 0;

int pulsesPerRevolution = 6; ///< Number of magnets on each wheel
//int frontPulsesPerRevolution = 6;
float divisor = 0.0; ///< Divisor calculated to turn absolute pulse count into rps
float divisorFront=0.0;
float rpsPublishPeriod = 13.0;///< Period (in ms) for publishing arduinoData messages, 70hz
time_t rpsPublishTime; ///< Time that the last message was published

//int counter=0;

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


//endpoints and neutral for each servo
int steerSrvNeutralUs = 1500;
int throttleSrvNeutralUs = 1500;
int frontBrakeSrvNeutralUs = 1500;
unsigned long timeOfLastServo = 0;
unsigned long servoTimeoutMs = 100;

Servo steerSrv;
Servo throttleSrv;
Servo frontBrakeSrv;

int castleLinkPeriod = 200; //get info at 2 Hz
char castlLinkDeviceID = 0; ///< ESC Device ID (set to default)
char castleLinkRegisters[] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; //< ESC data register numbers
char castleLinkData[2*sizeof(castleLinkRegisters)]; ///< each register is 2 bytes
unsigned long timeOfCastleLinkData = 0;

String errorMsg = "";
/**
* @brief Sets up all parameters, attaches interrupts, and initializes rosserial objects
*/
void setup()
{
  //pinMode(triggerPin, OUTPUT);
  //configureTriggerTimers();
  
  pinMode(rightRearRotationPin, INPUT); //right rear
  pinMode(leftRearRotationPin, INPUT); //left rear
  pinMode(rightFrontRotationPin, INPUT); //right front
  pinMode(leftFrontRotationPin, INPUT); //left rear
  digitalWrite(rightRearRotationPin,HIGH);
  digitalWrite(leftRearRotationPin,HIGH);
  digitalWrite(rightFrontRotationPin,HIGH);
  digitalWrite(leftFrontRotationPin,HIGH);

  pinMode(runStopPin, INPUT);
  digitalWrite(runStopPin, HIGH);
  
  configureRcInput();
  
  rpsPublishTime = 0;
  //attach all interrupts for rpm sensors
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
  Serial3.setTimeout(10);
  
  //clear serial link command buffer
  for(int i = 0; i < 5; i++)
  {
    Serial3.write('0x00');
  }

  // Setup input captures
  cap_ovr.config(SERVO_TIME_WINDOW);
  cap_steer.config(SERVO_TIME_WINDOW);
  cap_thr.config(SERVO_TIME_WINDOW);

  //R1 0.96  KOhm
  //R2 1.99  KOhm
  //analogVoltageScaler0 = 0.0049*(2.95/1.99);

  //R1 2.18  KOhm
  //R2 5.07  KOhm
  //analogVoltageScaler1 = 0.0049*(7.25/2.18);

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
      //parse servo message and set the servos if at least 7 bytes available (6 payload + \n)
      if(msgType == 's')
      {
        char buf[7];
        if(Serial.readBytes(buf, 7) == 7)
        {
          short steer = ((short)buf[0]<<8) + buf[1]&0xFFFF;
          short throttle = ((short)buf[2]<<8) + buf[3]&0xFFFF;
          short frontBrake = ((short)buf[4]<<8) + buf[5]&0xFFFF;
          
          timeOfLastServo = millis();
          
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

  
  if(rpsPublishTime+rpsPublishPeriod < millis())
  {
    rpsPublishTime = millis();
    //++counter;
    //divisor = (pulsesPerRevolution*(elapsed/1000.0));
    //divisorFront = (frontPulsesPerRevolution*(elapsed/1000.0));
    
    float leftFront, rightFront, leftBack, rightBack;  
    getrps(leftFront, rightFront, leftBack, rightBack);

    //float servoVoltage = analogRead(0)*analogVoltageScaler0;
    //float cameraVoltage = analogRead(1)*analogVoltageScaler1;
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

  if(errorMsg.length())
  {
    Serial.print("#e");
    Serial.print(errorMsg);
    Serial.print('\n');
    errorMsg = "";
  }
}

/**
 * @brief Configures the ATmega's timers for the desired camera frame rate
 * @note See http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts for details.
 */
//void configureTriggerTimers()
//{
//  noInterrupts(); //disable global interrupts
/*
  TCCR3A=0; //shut off timers
  TCCR3B = 0; //shut off timers
  int cnt = (int)(1.0/((2.0*triggerFPS)*0.000016) - 1.0); //compute new overflow value. 2FPS to turn bit on and off at twice the framerate
  OCR3A = cnt;  //set the compare register to this computed overflow value
  TCCR3B|=(1<<WGM32); //set timer to compare function (isntead of default overflow)
  TCCR3B |= (1<<CS32); //set prescaler to 1/256 
  TIMSK3 = (1<<OCIE3A); //enable compare interrupts

  */
//  interrupts(); //enable global interrupts
//}

void configureRcInput()
{
  noInterrupts();
  //timer counter for pin 3
  NVIC_EnableIRQ(TC7_IRQn);
  //enable pin
  pmc_enable_periph_clk(ID_TC7);
  
  //timer counter for pin 4
  NVIC_EnableIRQ(TC6_IRQn);
  //enable pin
  pmc_enable_periph_clk(ID_TC6);
  
  /*TCCR4A = 0;
  TCCR4B = 0;
  TCCR5A = 0;
  TCCR5B = 0;
  
  TCCR4B |= 1<<CS41;  //prescaler set to 1/8
  TCCR4B |= 1<<ICNC4; //Noise canceler on
  TCCR4B |= 1<<ICES4; //Rising edge trigger
  TCCR5B |= 1<<CS41;  //prescaler set to 1/8
  TCCR5B |= 1<<ICNC4; //Noise canceler on
  TCCR5B |= 1<<ICES4; //Rising edge trigger
  
  TIMSK4 |= 1<<ICIE4; //Input captuer interrupt on
  TIMSK5 |= 1<<ICIE5; 
  */
  //Port L is already setup as input.
  
  interrupts();
}

void getRcWidths(uint32_t &rc_1, uint32_t &rc_2, uint32_t &rc_3)
{
  uint32_t ret, duty, period, pulses;
  ret = cap_ovr.get_duty_period_and_pulses(duty, period, pulses);
  rc_1 = (double)duty/(double)cap_ovr.ticks_per_usec();
  ret = cap_steer.get_duty_period_and_pulses(duty, period, pulses);
  rc_2 = (double)duty/(double)cap_steer.ticks_per_usec();
  ret = cap_thr.get_duty_period_and_pulses(duty, period, pulses);
  rc_3 = (double)duty/(double)cap_thr.ticks_per_usec();     
}

void getrps(float &w0, float &w1, float &w2, float &w3)
{
  unsigned long w0temp, w1temp, w2temp, w3temp;
  unsigned long w0t, w1t, w2t, w3t;
  //uint8_t oldSREG = SREG;
  noInterrupts();
  w0t = w0updatetime;
  w0temp = w0period;
  w1t = w1updatetime;
  w1temp = w1period;
  w2t = w2updatetime;
  w2temp = w2period;
  w3t = w3updatetime;
  w3temp = w3period;
  //SREG = oldSREG;
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

/*
ISR(TIMER4_CAPT_vect)
{
  char temp = PINL & 0x01;
  if (temp)
  {
    TCCR4B &= ~(1<<ICES4);  //Falling edge trigger
    rc_risingEdge4 = ICR4;
  }
  else
  {
    TCCR4B |= 1<<ICES4;     //Rising edge trigger
    rc_width4 = ICR4 - rc_risingEdge4;
  }
}

ISR(TIMER5_CAPT_vect)
{
  char temp = PINL & 0x02;
  if (temp)
  {
    TCCR5B &= ~(1<<ICES5);  //Falling edge trigger
    rc_risingEdge5 = ICR5;
  }
  else
  {
    TCCR5B |= 1<<ICES5;     //Rising edge trigger
    rc_width5 = ICR5 - rc_risingEdge5;
  }
}
*/

/**
 * @brief Interrupt callback for camera trigger timer
 */
//ISR(TIMER3_COMPA_vect)
//{
//  digitalWrite(triggerPin, !digitalRead(triggerPin));
//}

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

char castleChecksum(char* msg)
{
  return (0 - (msg[0] + msg[1] + msg[2] + msg[3]));
}

