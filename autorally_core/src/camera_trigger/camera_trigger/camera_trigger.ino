

/**********************************************
 * @file camera_trigger,ino
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 20, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Triggers the cameras
 *
 ***********************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

int triggerPin = 3; ///< Pin number on the Arduino Micro used to trigger the cameras
int triggerFPS = 40; ///< default trigger rate

unsigned long time; ///< Time that the last message was published
unsigned long elapsed; ///< Calculated elapsed time since last transmission

unsigned long pps; ///< Time that last pps message was recived
int ppsCount=0; ///< count of received pps pulses
float dataPublishPeriod = 500; ///< Period for data to be sent back to computer

/**
* @brief Sets up all parameters and attaches interrupts
*/
void setup()
{
  pinMode(triggerPin, OUTPUT);
  configureTriggerTimers();
  //attach all interrupt for incoming pps
  attachInterrupt(2, int0, RISING);
   
  time = 0;
  pps = 0;
  
  Serial.begin(115200);
}

/**
* @brief Main body of program, processes data from USB, sends diagnostic info over USB.
*/
void loop()
{
  //parse incoming fps trigger requests from computer
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
    time = millis();
   
    //if received a pps within last second
    Serial.print("#pps:");
    Serial.println(ppsCount);
    
    Serial.print("#fps:");
    Serial.println(triggerFPS);
    
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

/**
 * @brief Interrupt callback for camera trigger timer
 */
ISR(TIMER3_COMPA_vect)
{
  digitalWrite(triggerPin, !digitalRead(triggerPin));
}

/**
* @brief Interrupt service routine 0 (for incoming pps signal). Currently doesn't do anything
*/
void int0()
{
  pps = millis();
  ++ppsCount;
}


