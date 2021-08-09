/*
*
* This is the example file for the Atlas Aerospace 
* quaternion code.
* 
* This code is designed to prevent gimbal lock.
* 
* Quaternions are a way to prevent gimbal lock by
* adding a virtual 4th gimbal.
* Rather than euler angles which are three numbers:
* -x
* -y
* -z
* 
* Quaternions use four numbers:
* -w
* -i
* -j
* -k
*
*
* This code is designed to be easy to use with two 
* functions: Update() and To_Euler(). It is perfect
* for a project where you may run into gimbal lock
* it should work with any IMU using x, y and z which 
* outputs an angular rate.
* 
* 
* Written by:
*
* Atlas Aerospace 03/08/2021
*
*
* My YouTube:
*
* https://www.youtube.com/channel/UCWd6oqc8nbL-EX3Cxxk8wFA
*/

#include "Quat.h"

// This must be defined in the global space, it is the base quaternions
// which gets modified.
Quat base = {1, 0, 0, 0};

// Variables to calculate the delta time.
double current_time, previous_time, deltatime;

// This setup function will depend on your gyro and also the baud rate 
// for serial.
void setup(){
  Serial.begin(115200);

  your_gyro.begin();
}

// This is the main loop which gets the delta time, gyro rate and processes
// the gyro input.
void loop(){

  // Gets current time through micros as micros is more accurate than millis
  // Delta time is difference in time so it is current time - previous time
  current_time = micros() / 1000000.0f;
  deltatime = current_time - previous_time;

  // This creates a gyro vector to hold all of the gyro variables. Vect has
  // three variables: x, y and z.
  Vect gyro;

  // Your gyro code will vary this is just an example of what it may look at.
  your_gyro.update();

  // Sets the gyro variables to the gyro rate.
  gyro.x = your_gyro.getGyroX();
  gyro.y = your_gyro.getGyroY();
  gyro.z = your_gyro.getGyroZ();

  // This is where the beginning of the quaternions happens. First, we have
  // to convert the gyro rates to radians as that is the input that the library
  // expect. Radians are the standard unit.
  gyro = gyro.To_Radians();

  // Here we update the base, the base is essentially the starting position for 
  // quaternions. Every loop, this updates with the gyro rate. 
  base = base.Update(gyro, deltatime);

  // Now that we have updated the quaternion values, we convert these to nice 
  // and easy euler angles. This is now output in radians.
  gyro = base.To_Euler();

  // Finally, we convert the gyro back to degrees although, you may wish to
  // leave this step out depending on the rest of your loop.
  gyro = gyro.To_Degrees();

  // This prints the data to see if it is working.
  Serial.print(" Quat Output: "); Serial.print(gyro.x);
  Serial.print(" , "); Serial.print(gyro.y);
  Serial.print(" , "); Serial.println(gyro.z);

  // We then have to set previous time to current time at the very end of the
  // loop. Without this, the delta time would not be accurate and the whole
  // program would not work.
  previous_time = current_time;
  
}
