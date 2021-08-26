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

Quat Quat::Update(Vect v, double dt) {
    Quat q;
    
    Quat q_half = {w * 0.5f, i * 0.5, j * 0.5f, k * 0.5,};
    
    q.w = -q_half.i * v.x - q_half.j * v.y - q_half.k * v.z;
    q.i = q_half.w * v.x - q_half.k * v.y + q_half.j * v.z;
    q.j = q_half.k * v.x + q_half.w * v.y - q_half.i * v.z;
    q.k = -q_half.j * v.x + q_half.i * v.y + q_half.w * v.z;

    w += q.w * dt;
    i += q.i * dt;
    j += q.j * dt;
    k += q.k * dt;

    double norm = sqrt(w * w + i * i + j * j + k * k);
 
    Quat b = {w, i, j, k};

    b /= norm;
    
    return b;
}

Vect Quat::To_Euler() {
    Vect ypr;

    ypr.x = atan2(2.0f * (i * j + w * k), w * w + i * i - j * j - k * k);
    ypr.y = -asin(2.0f * (i * k - w * j));
    ypr.z = atan2(2.0f * (w * i + j * k), w * w - i * i - j * j + k * k);

    return ypr;
}
