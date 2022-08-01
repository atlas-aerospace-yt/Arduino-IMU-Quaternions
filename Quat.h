#pragma once
#include "math.h"
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067

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

    float w, i, j, k, norm;

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
