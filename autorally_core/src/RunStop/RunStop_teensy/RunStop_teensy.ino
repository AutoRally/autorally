/**
* Software License Agreement (BSD License)
* Copyright (c) 2018, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

/**
* @file RunStop.ino
* @author Cory Wacht <cwacht3@gatech.edu>
* @date 5/17/2013
* @brief the button input of the emergency stop device and outputs corresponding data.
*         RED = motion disabled  YELLOW = caution  GREEN = motion enabled
**/

int initial = 0; //State starts at RED
int state = 0;
char s0[ ] = "#estopstate:RED";
char s2[ ] = "#estopstate:GREEN";

void setup() {
  Serial.begin(57600);
  pinMode(0,INPUT); //red mushroom button
  pinMode(1,INPUT); // green momentary button
  pinMode(2,INPUT); // red momentary button
}

void loop() {
  int kill = digitalRead(0);
  int green = digitalRead(1);
  int red = digitalRead(2);

  if (kill == LOW || red == LOW || initial == 0) { //red mushroom or red button or initial state
    state = 0;
    initial = 1;
  }else if(green == HIGH){ //Green Button
    state = 2;
  }

  if (state == 0) {     //red mushroom or red button or initial state
     Serial.println(s0);
     initial = 1;
  } else if (state == 2) { //green momentary is pressed
     Serial.println(s2);
  }
  delay(50);
}
