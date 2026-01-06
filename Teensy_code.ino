#include <Servo.h>

Servo ESC1; 
Servo ESC2;     // create servo object to control the ESC
Servo ESC3;     // create servo object to control the ESC
Servo ESC4;     // create servo object to control the ESC
    // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(8,1000,2000);
  ESC3.attach(7,1000,2000);
  ESC4.attach(6,1000,2000);
}

void loop() {
  potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC1.write(potValue);    // Send the signal to the ESC
  ESC2.write(potValue);    // Send the signal to the ESC
  ESC3.write(potValue);    // Send the signal to the ESC
  ESC4.write(potValue);    // Send the signal to the ESC
}
