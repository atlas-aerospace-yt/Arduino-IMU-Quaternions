/*
*
* This is the header file for the Atlas Aerospace 
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

#pragma once
#include "math.h"
#define PI 3.14159

struct Vect {
    double x, y, z;

    Vect To_Radians();
    Vect To_Degrees();
};

struct Quat {
    double w, i, j, k;


    Quat operator/= (float f) {
        Quat q;

        q.w = w / f;
        q.i = i / f;
        q.j = j / f;
        q.k = k / f;

        return q;
    }

    Quat Update(Vect v, double dt);
    Vect To_Euler();
};
