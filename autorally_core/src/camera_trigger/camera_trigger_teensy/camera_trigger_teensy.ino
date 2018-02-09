

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

int triggerPin = 1; ///< Pin number on the Tensy LC used to trigger the cameras
int PPSPin = 2;     ///< Pin number for pps input
int triggerFPS = 40; ///< default trigger rate

unsigned long time; ///< Time that the last message was published
unsigned long elapsed; ///< Calculated elapsed time since last transmission

unsigned long pps; ///< Time that last pps message was recived
int ppsCount=0; ///< count of received pps pulses
float dataPublishPeriod = 500; ///< Period for data to be sent back to computer

IntervalTimer triggerTimer;

/**
* @brief Sets up all parameters and attaches interrupts
*/
void setup()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(13, OUTPUT);
  unsigned long timerInterval = 1000000 / (triggerFPS*2);
  triggerTimer.begin(blinkTriggerOutput, timerInterval);
  //configureTriggerTimers();
  //attach all interrupt for incoming pps
  attachInterrupt(digitalPinToInterrupt(PPSPin), int0, RISING);
   
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
        unsigned long timerInterval = 1000000 / (triggerFPS*2);
        triggerTimer.update(timerInterval);
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

    digitalWrite(13, !digitalRead(13));
  }
}

/**
 * @brief Interrupt callback for camera trigger timer
 */
void blinkTriggerOutput()
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


