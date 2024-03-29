#include "Quat.h"
#include "Arduino.h"

Vect Vect::To_Radians() {
  Vect ypr;

  ypr.x = x * (PI / 180.0f);
  ypr.y = y * (PI / 180.0f);
  ypr.z = z * (PI / 180.0f);

  return ypr;
}

Vect Vect::To_Degrees() {
  Vect ypr;

  ypr.x = x * (180.0f / PI);
  ypr.y = y * (180.0f / PI);
  ypr.z = z * (180.0f / PI);

  return ypr;
}


// Quaternions from angular rate using madgwick paper
Quat Quat::Update(Vect v, float dt) {

  Quat b = {0, v.x, v.y, v.z};
  Quat r = {w, i, j, k};

  Quat q = r * 0.5 * b;
  Quat n = r + (q * dt);

  n.norm = sqrt(n.w * n.w + n.i * n.i + n.j * n.j + n.k * n.k);

  n = n / n.norm;

  return n;
}

// Euler angles from quaternions
Vect Quat::To_Euler()
{
  Vect ypr;

  // X-Axis calculations
  float x_one  = 2 * i * j - 2 * w * k;
  float x_two = 2 * w * w + 2 * i * i - 1;
  ypr.x = atan2(x_one, x_two);

  // Y_Axis calculations
  float y = 2 * i * k + 2 * w * j;
  if (y > 1.0f){
    ypr.y = -asin(PI * 2 / y); // uses 90 as a failsafe
  }
  else{
    ypr.y = -asin(y);
  }

  // Z-Axis calculations
  float z_one = 2 * j * k - 2 * w * i;
  float z_two = 2 * w * w + 2 * k * k - 1;
  ypr.z = atan2(z_one, z_two);

  return ypr;
}
