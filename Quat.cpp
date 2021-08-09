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

    q.w = 0.5f * (-i * v.x - j * v.y - k * v.z);
    q.i = 0.5f * (w * v.x - k * v.y + j * v.z);
    q.j = 0.5f * (k * v.x + w * v.y - i * v.z);
    q.k = 0.5f * (-j * v.x + i * v.y + w * v.z);


    w += q.w * dt;
    i += q.i * dt;
    j += q.j * dt;
    k += q.k * dt;

    Quat b = {w, i, j, k};

    double norm = sqrt(b.w * b.w + b.i * b.i + b.j * b.j + b.k * b.k);

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
