# Building A Controller Project

## Implementing body rate control in C++

These are the essential lines of the implementation:
```
  V3F pqr_err = pqrCmd - pqr;
  momentCmd = kpPQR * V3F{Ixx, Iyy, Izz} * pqr_err;
```

This takes into account the inertia and the respective control parameters.

## Implement roll pitch control in C++

These are the essential lines of the implementation:

```
  float c_d = -collThrustCmd / mass;

  if (collThrustCmd > 0.0f) {
    float target_R13 =
        CONSTRAIN(accelCmd.x / c_d, -maxTiltAngle, maxTiltAngle);
    float target_R23 =
        CONSTRAIN(accelCmd.y / c_d, -maxTiltAngle, maxTiltAngle);
    float v[9] = {R(1, 0), -R(0, 0), 0.0f, 
                  R(1, 1), -R(0, 1), 0.0f,
                     0.0f,     0.0f, 0.0f};
    Mat3x3F rot_mat1 = Mat3x3F(v) / R(2, 2);
    V3F b =
        kpBank * V3F{target_R13 - R(0, 2), target_R23 - R(1, 2), 0.0f};
    pqrCmd = rot_mat1 * b;

  } else {
    pqrCmd.x = 0.0f;
    pqrCmd.y = 0.0f;
  }
```

We constrain the maximal tilding angle of the acceleration command. The
2d rotation to account for the transformation from local to body frame is
implemented as a 3d rotation matrix but with zero elements in the 3th row. The
collective thrust is divided by the drones mass to get the acceleration.

## Implement altitude controller in C++

These are the essential lines of the implementation:

```
  float z_err = posZCmd - posZ;
  integratedAltitudeError += z_err * dt;
  float vel_z_err = velZCmd - velZ;
  float b_z = attitude.RotationMatrix_IwrtB()(2,2);
  float pTerm = kpPosZ * z_err;
  float dTerm = kpVelZ * vel_z_err;
  float iTerm = KiPosZ * integratedAltitudeError;
  float u1Bar = pTerm + iTerm + dTerm + accelZCmd;
  float thrustAcc = (u1Bar - CONST_GRAVITY) / b_z;
  float thrust = -mass * thrustAcc ;
```

Noteworthy here is the `integratedAltitudeError` update that helps dampen the system. To get a thrust, the computed acceleration is multiplied with the drones mass in the end.

## Implement lateral position control in C++


These are the essential lines of the implementation:

```
  V3F pos_err = posCmd - pos;
  if (velCmd.mag() > maxSpeedXY ) {
    velCmd = velCmd.norm() * maxSpeedXY;
  }

  V3F vel_err = velCmd - vel;
  accelCmd = kpPosXY * pos_err + kpVelXY * vel_err + accelCmd;
  if (accelCmd.mag() > maxAccelXY) {
    accelCmd = accelCmd.norm() * maxAccelXY;
  }
```

Of note here is the check to make sure we don't exceed maximal speed/acceleration in the XY directions.

## Implement yaw control in C++

```
  float yaw_cmd = fmodf(yawCmd, 2 * F_PI);
  float yaw_error = yaw_cmd - yaw;
  yaw_error = fmodf(yaw_error, 2 * F_PI);
  float yawRateCmd = kpYaw * yaw_error;
```

This implementation is straight forward. Of note is the assertion to make sure that the commanded yaw angle stays with the 0-2pi range.

## Implement calculating the motor commands given commanded thrust and moments in C++

These are the essential lines of the implementation:
```
  float l = L / sqrtf(2.f);
  float p_bar = momentCmd.x / l;
  float q_bar = momentCmd.y / l;
  float r_bar = -momentCmd.z / kappa;
  float c_bar = collThrustCmd;


  cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;
  cmd.desiredThrustsN[1] = (-p_bar + q_bar - r_bar + c_bar) / 4.f;
  cmd.desiredThrustsN[2] = (p_bar - q_bar - r_bar + c_bar) / 4.f;
  cmd.desiredThrustsN[3] = (-p_bar -q_bar + r_bar + c_bar) / 4.f;
```

This is essentially the code shown in the python solution. Important is to scale the z-moment command with the inverse of the kappa param to get the z-axis thrust.The x/y moment commands need to scaled with the inverse of the projected arm length of the drone.
