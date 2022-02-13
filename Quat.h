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
    float x, y, z;

    Vect operator/ (float f) {
      Vect v;

      v.x = x / f;
      v.y = y / f;
      v.z = z / f;

      return v;
    }

    Vect operator* (float f){
      Vect v;

      v.x = x * f;
      v.y = y * f;
      v.z = z * f;

      return v;
    }
      
    Vect To_Radians();
    Vect To_Degrees();
};

struct Quat {
  
    float w, i, j, k;
    
    Quat operator/ (float f) {
        Quat q;

        q.w = w / f;
        q.i = i / f;
        q.j = j / f;
        q.k = k / f;

        return q;
    }
    
    Quat operator* (Quat q){
      Quat r;

      r.w = w * q.w - i * q.i - j * q.j - k * q.k;
      r.i = w * q.i + i * q.w + j * q.k - k * q.j;
      r.j = w * q.j - i * q.k + j * q.w + k * q.i;
      r.k = w * q.k + i * q.j - j * q.i + k * q.w;
      
      return r;
    }

    Quat operator* (float f){
      Quat q;

      q.w = w * f;
      q.i = i * f;
      q.j = j * f;
      q.k = k * f;

      return q;
    }
    
    Quat operator+ (Quat q){
      Quat r;

      r.w = w + q.w;
      r.i = i + q.i;
      r.j = j + q.j;
      r.k = k + q.k;

      return r;
    }

    Quat operator- (Quat q){
      Quat r;

      r.w = w - q.w;
      r.i = i - q.i;
      r.j = j - q.j;
      r.k = k - q.k;

      return r;
    }

    Quat operator+ (float f){
      Quat r;

      r.w = w + f;
      r.i = i + f;
      r.j = j + f;
      r.k = k + f;

      return r;
    }
    
    Quat operator- (float f){
      Quat r;

      r.w = w - f;
      r.i = i - f;
      r.j = j - f;
      r.k = k - f;

      return r;
    }

    Quat Update(Vect v, float dt);
    Vect To_Euler();
};
