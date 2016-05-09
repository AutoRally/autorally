

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

#include <avr/io.h>
#include <avr/interrupt.h>

#define MAX_MEASUREMENT_AGE 400  ///< Zero the reading at 400 ms

int triggerPin = 40;
int triggerFPS = 40;

float analogVoltageScaler0; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs
float analogVoltageScaler1; ///< Scale the analog reading 0-1024 to 0-5 volts, value from ADC specs

volatile int rpscount0; ///< Counts the ticks from the Hall Effect sensor 0
volatile int rpscount1; ///< Counts the ticks from the Hall Effect sensor 1
volatile int rpscount2; ///< Counts the ticks from the Hall Effect sensor 2
volatile int rpscount3; ///< Counts the ticks from the Hall Effect sensor 3

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

int pulsesPerRevolution = 6; ///< Number of magnets on each wheel
//int frontPulsesPerRevolution = 6;
float divisor = 0.0; ///< Divisor calculated to turn absolute pulse count into rps
float divisorFront=0.0;
float dataPublishPeriod = 14.0;///< Period (in ms) for publishing arduinoData messages, 70hz
unsigned long time; ///< Time that the last message was published
unsigned long diagTime; ///< Time that the last diagnostic message was published
unsigned long elapsed; ///< Calculated elapsed time since last transmission
unsigned long diagElapsed; ///< Calculated elapsed time since last transmission
int counter=0;

/*ros::NodeHandle  nh; ///< Nodehandle to publish data into system
autorally_core::arduinoData msg; ///< Sensor data message to publish
diagnostic_msgs::DiagnosticArray diag; ///< Diagnostics message to publish
diagnostic_msgs::DiagnosticStatus stat; ///< Diagnostic status to publish
diagnostic_msgs::KeyValue info; ///< Diagnostics key value pair
ros::Publisher publisher("arduinoData", &msg); ///< ArduinoData publisher
ros::Publisher diagPublisher("diagnostics", &diag); ///< ArduinoData publisher*/

/**
* @brief Sets up all parameters, attaches interrupts, and initializes rosserial objects
*/
void setup()
{
  pinMode(triggerPin, OUTPUT);
  configureTriggerTimers();
  
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);
  digitalWrite(20,HIGH);
  digitalWrite(21,HIGH);
  
  configureRcInput();
  
  time = 0;
  //attach all interrupts for rpm sensors
  attachInterrupt(2, int0, RISING);
  attachInterrupt(3, int1, RISING);
  attachInterrupt(4, int2, RISING);
  attachInterrupt(5, int3, RISING);

  rpscount0 = 0;
  rpscount1 = 0;
  rpscount2 = 0;
  rpscount3 = 0;
  
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


  Serial.begin(115200);


  //R1 0.96  KOhm
  //R2 1.99  KOhm
  analogVoltageScaler0 = 0.0049*(2.95/1.99);

  //R1 2.18  KOhm
  //R2 5.07  KOhm
  analogVoltageScaler1 = 0.0049*(7.25/2.18);

}

/**
* @brief Main body of program, gathers and publishes sensor data.
*/
void loop()
{
  if(Serial.available() && Serial.read() == '#')
  {
      char buffer[10];
      int numBytesRead = Serial.readBytesUntil(':', buffer, 10);
      if(numBytesRead < 10)
        buffer[numBytesRead] = '\0';
      if(strcmp(buffer, "fps") == 0)
      {
        triggerFPS = Serial.parseInt();
        configureTriggerTimers();
      }
  }
  elapsed = millis()-time;
  if(elapsed >= dataPublishPeriod)
  {
    counter++;
    time = millis();
    //divisor = (pulsesPerRevolution*(elapsed/1000.0));
    //divisorFront = (frontPulsesPerRevolution*(elapsed/1000.0));
    
    float leftFront, rightFront, leftBack, rightBack;
    
    getrps(leftFront, rightFront, leftBack, rightBack);

    //float servoVoltage = analogRead(0)*analogVoltageScaler0;
    //float cameraVoltage = analogRead(1)*analogVoltageScaler1;

    Serial.print("#wheels:");
    Serial.print(leftFront,3);
    Serial.print(",");
    Serial.print(rightFront,3);
    Serial.print(",");
    Serial.print(leftBack,3);
    Serial.print(",");
    Serial.print(rightBack,3);
    Serial.print("\n");
    
    //Serial.print("servo:");
    //Serial.print(servoVoltage,2);
    //Serial.print("\n");
    
    //Serial.print("camera:");
    //Serial.print(cameraVoltage,2);
    //Serial.print("\n");

    Serial.print("rc:");
    Serial.print(rc_width4);
    Serial.print(",");
    Serial.println(rc_width5);
  }

}

/**
 * @brief Configures the ATmega's timers for the desired camera frame rate
 * @note See http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts for details.
 */
void configureTriggerTimers()
{
  cli(); //disable global interrupts
  TCCR3A=0; //shut off timers
  TCCR3B = 0; //shut off timers
  int cnt = (int)(1.0/((2.0*triggerFPS)*0.000016) - 1.0); //compute new overflow value. 2FPS to turn bit on and off at twice the framerate
  OCR3A = cnt;  //set the compare register to this computed overflow value
  TCCR3B|=(1<<WGM32); //set timer to compare function (isntead of default overflow)
  TCCR3B |= (1<<CS32); //set prescaler to 1/256 
  TIMSK3 = (1<<OCIE3A); //enable compare interrupts
  sei(); //enable global interrupts
}

void configureRcInput()
{
  cli();
  TCCR4A = 0;
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
  
  //Port L is already setup as input.
  
  sei();
}

void getrps(float &w0, float &w1, float &w2, float &w3)
{
  unsigned long w0temp, w1temp, w2temp, w3temp;
  unsigned long w0t, w1t, w2t, w3t;
  uint8_t oldSREG = SREG;
  cli();
  w0t = w0updatetime;
  w0temp = w0period;
  w1t = w1updatetime;
  w1temp = w1period;
  w2t = w2updatetime;
  w2temp = w2period;
  w3t = w3updatetime;
  w3temp = w3period;
  SREG = oldSREG;
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

/**
 * @brief Interrupt callback for camera trigger timer
 */
ISR(TIMER3_COMPA_vect)
{
  digitalWrite(triggerPin, !digitalRead(triggerPin));
}

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
