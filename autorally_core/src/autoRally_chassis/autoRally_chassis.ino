

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

#define MAX_MEASUREMENT_AGE 400  ///< Zero the reading at 400 ms

//int triggerPin = 40;
//int triggerFPS = 40;

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

volatile unsigned int rc_risingEdge4;  ///< Record the rising edge of rc signal on IC4
volatile unsigned int rc_width4;       ///< Record the pulse width of the rc signal on IC4
volatile unsigned int rc_risingEdge5;  ///< Record the rising edge time of rc signal on IC5
volatile unsigned int rc_width5;       ///< Record the pulse width of the rc signal on IC5

volatile unsigned int rc_width6 = 0;

int pulsesPerRevolution = 6; ///< Number of magnets on each wheel
//int frontPulsesPerRevolution = 6;
float divisor = 0.0; ///< Divisor calculated to turn absolute pulse count into rps
float divisorFront=0.0;
float rpsPublishPeriod = 1000.0;///< Period (in ms) for publishing arduinoData messages, 70hz
time_t recTime; ///< Time that the last message was published
//unsigned long diagTime; ///< Time that the last diagnostic message was published
//unsigned long elapsed; ///< Calculated elapsed time since last transmission
//unsigned long diagElapsed; ///< Calculated elapsed time since last transmission
int counter=0;

int steerReadPin = 3;
int throttleReadPin = 4;
int manualModePin = 5;
int runStopPin = 6;

int frontBrakePin = 10;
int throttlePin = 11;
int steerPin = 12;

//endpoints and neutral for each servo
int steerSrvNeutralUs = 1500;
int throttleSrvNeutralUs = 1500;
int frontBrakeSrvNeutralUs = 1500;
unsigned long timeOfLastServo = 0;
unsigned long servoTimeoutMs = 250;

Servo steerSrv;
Servo throttleSrv;
Servo frontBrakeSrv;

int castleLinkPeriod = 2000; //get info once every 1000ms
char castlLinkDeviceID = 0;
char castleLinkRegisters[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
char castleLinkData[2*sizeof(castleLinkRegisters)];
unsigned long timeOfCastleLinkData = 0;

String errorMsg = "";
/**
* @brief Sets up all parameters, attaches interrupts, and initializes rosserial objects
*/
void setup()
{
  //pinMode(triggerPin, OUTPUT);
  //configureTriggerTimers();
  
  pinMode(18, INPUT); //right rear
  pinMode(19, INPUT); //left rear
  pinMode(20, INPUT); //right front
  pinMode(21, INPUT); //right rear
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);
  digitalWrite(20,HIGH);
  digitalWrite(21,HIGH);

  pinMode(runStopPin, INPUT);
  digitalWrite(runStopPin, HIGH);
  
  configureRcInput();
  
  recTime = 0;
  //attach all interrupts for rpm sensors
  attachInterrupt(18, int0, RISING);
  attachInterrupt(19, int1, RISING);
  attachInterrupt(20, int2, RISING);
  attachInterrupt(21, int3, RISING);
  
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
  
  //clear serial link command buffer
  for(int i = 0; i < 5; i++)
  {
    Serial3.write('0x00');
  }
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
  //make sure we are framed at the beginning of a message
  if(Serial.available() && Serial.read() == '#')
  {
    char msgType = Serial.read();
    //parse servo message and set the servos if at least 7 bytes available (6 payload + \n)
    if(msgType == 's' && Serial.available() >= 7)
    {
      char buf[7];
      if(Serial.readBytes(buf, 7) == 7)
      {
        short steer = buf[0]<<8 + buf[1]&0xFFFF;
        short throttle = buf[2]<<8 + buf[3]&0xFFFF;
        short frontBrake = buf[4]<<8 + buf[5]&0xFFFF;
        
        timeOfLastServo = millis();
        
        steerSrv.writeMicroseconds(steer);
        throttleSrv.writeMicroseconds(throttle);
        frontBrakeSrv.writeMicroseconds(frontBrake);
      }   
    }
  }

  if( timeOfLastServo+servoTimeoutMs < millis())
  {
    steerSrv.writeMicroseconds(steerSrvNeutralUs);
    throttleSrv.writeMicroseconds(throttleSrvNeutralUs);
    frontBrakeSrv.writeMicroseconds(frontBrakeSrvNeutralUs);
  }
 
  if(recTime+rpsPublishPeriod < millis())
  {
    counter++;
    recTime = millis();
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

    Serial.print("#r");
    Serial.print(rc_width4);
    Serial.print(",");
    Serial.print(rc_width5);
    Serial.print(",");
    Serial.print(rc_width6);
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


void TC7_Handler()
{
  long dummy=REG_TC2_SR1;
}

void TC8_Handler()
{
  long dummy=REG_TC2_SR2;
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
* @brief Interrupt service routine 0 (front left rpm sensor)
*/
void int0()
{
  unsigned long t = micros();
  w0period = t - w0prevtimer;
  w0prevtimer = t;
  w0updatetime = millis();
}

/**
* @brief Interrupt service routine 1 (front right rpm sensor)
*/
void int1()
{
  unsigned long t = micros();
  w1period = t - w1prevtimer;
  w1prevtimer = t;
  w1updatetime = millis();
}

/**
* @brief Interrupt service routine 2 (back left rpm sensor)
*/
void int2()
{
  unsigned long t = micros();
  w2period = t - w2prevtimer;
  w2prevtimer = t;
  w2updatetime = millis();
}

/**
* @brief Interrupt service routine 3 (back right rpm sensor)
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
  
  for(int i = 0; i < sizeof(castleLinkRegisters); i++)
  {
    request[0] = (0x1<<7) + castlLinkDeviceID;
    request[1] = castleLinkRegisters[i];
    request[2] = 0;
    request[3] = 0;
    request[4] = castleChecksum(request);

    Serial3.write(request, 5);
    //Serial.print("request "); 
    //for(int j = 0; j < 5; j++)
    //{
    //  Serial.print((unsigned int)request[j]);
    //  Serial.print(" ");
    //}
    //Serial.println(); 

    //read 3 byte responses
    int bytesRead = Serial3.readBytes(bufRec, 3);
    if(bytesRead == 3)
    {
      //for(int i = 0; i < 3; i++)
      //{
      //  Serial.print((unsigned int)bufRec[i]);
      //  Serial.print(" ");
      //}

      //unsigned int reg = (bufRec[0]<<8) + bufRec[1]&0xFFFF;
      //Serial.print(reg);
      //Serial.print(" ");
      //float vol = ( reg/2042.0)*4.0;
      //Serial.print(vol);
      //make sure cheksum checks out
      //char checksum = (bufRec[0] + bufRec[1] + bufRec[2]);
      //Serial.print((int)checksum);
      //Serial.println();
      
      if( (char)(bufRec[0] + bufRec[1] + bufRec[2]) == 0)
      {
        if(bufRec[0] == 255 && bufRec[1] == 255) // error from serial link indicated by 0xFFFF
        {
          errorMsg += "castle link comm error on register " + ('0'+bufRec[0]) + ',';
          //invalid register of corrupted command
        } else
        {
          //bufReg[0] is register
          //bufReg[1] is value of register

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

