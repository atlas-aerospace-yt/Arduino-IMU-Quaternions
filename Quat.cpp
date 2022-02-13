/*
*
* This is the C++ file for the Atlas Aerospace 
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

Vect Vect::To_Radians(){
  Vect ypr;

  ypr.x = x * (PI / 180.0f);
  ypr.y = y * (PI / 180.0f);
  ypr.z = z * (PI / 180.0f);

  return ypr;
}

Vect Vect::To_Degrees(){
  Vect ypr;

  ypr.x = x * (180.0f / PI);
  ypr.y = y * (180.0f / PI);
  ypr.z = z * (180.0f / PI);

  return ypr;
}

Quat Quat::Update(Vect v, float dt) {

    Quat b = {0, v.x, v.y, v.z};
    Quat r = {w, i, j, k};

    Quat q = r * 0.5 * b; 
    Quat n = r + q * dt;
    
    return n;
}

Vect Quat::To_Euler() {
    Vect ypr;

    ypr.x = atan2(2 * i * j - 2 * w * k, 2 * w * w + 2 * i * i - 1);
    ypr.y = -asin(2 * i * k + 2 * w * j);
    ypr.z = atan2(2 * j * k - 2 * w * i, 2 * w * w + 2 * k * k - 1);
    
    return ypr; 
}
