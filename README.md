kalman-ios
==========

**DISCLAIMER**: I authored this repo some time ago. I no longer consider it a good reference implementation of an attitude filter. Were I to re-write it today the code would look different in many respects. It is entirely plausible that there are mistakes in the current implementation. The code is not maintained, nor is it unit-tested to the standard I would now demand. I would encourage developers to _look elsewhere for a better example_.

---

Author: Gareth, gcross [dot] code [at] icloud.com

---

Description:

This is an implementation of an Extended Kalman Filter (EKF) for orientation sensing using the accelerometer, gyroscope and (optionally) magnetometer of an iOS device. I use the multiplicative error state quaternion formulation described in:

“Attitude Error Representations for Kalman Filtering” - F. Landis Markley

With the following modifications: 
- The gyroscope biases are estimated offline in a calibration step. 
- Two external reference vectors are used instead of one (if the magnetometer is enabled).
- The accelerometer measurement covariance is scaled up when the measured acceleration deviates from one g.

In this style of filter the device orientation is represented as the product of two transforms, the nominal and error states. The nominal state is stored as a unit length quaternion, while the error state consists of 3 Euler angles. The quaternion representation is mathematically compact and numerically stable throughout all possible orientations (no singularities), rendering it suitable for use in aerospace applications. 

In the prediction step, the nominal state is integrated using 4th order Runge-Kutta with the angular velocities measured by the gyroscope. The error state is then determined by projecting the reference vectors (gravity and magnetic field) using the state quaternion, and calculating the residual between the measured and predicted vectors. Finally, the correction to the orientation is compounded onto the nominal state and the error estimate is zeroed (reset).

Video of the filter in action: [Link](http://www.youtube.com/watch?v=ijK2ndEGBXA "YouTube Video")

---

Implementation notes:

- The demo is written for iOS, but all of the filter components are in generic C++ and should run on other platforms without issue.
- The magnetometer ‘calibration’ employed is very simple, and does not adjust for variations in axis sensitivity or possible misalignment between axes. A more thorough implementation would use maximum likelihood estimation (MLE) to find these parameters.
- The magnetometer can be disabled, resulting in smoother operation of the filter (since the magnetometer is the primary source of measurement noise). In this mode, the yaw axis drifts without correction - which may be acceptable in instances where the filter does not run for extended periods, or if the gyroscope is very precise.


