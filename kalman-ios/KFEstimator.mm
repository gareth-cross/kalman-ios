//
//  KFEstimator.m
//  kalman-ios
//
//  Created by Gareth Cross on 12/27/2013.
//  Copyright (c) 2013 gareth. Apache 2 License.
//

#import "KFEstimator.h"
#include <mach/mach_time.h>

#include "AttitudeESKF.hpp"
#include "AttitudeMagCalib.hpp"
#include <deque>

using namespace Eigen;

static uint64_t getTime_ns()
{
  static mach_timebase_info_data_t s_timebase_info;
  
  //  get the time scale
  if (s_timebase_info.denom == 0) {
    mach_timebase_info(&s_timebase_info);
  }
  
  return ((mach_absolute_time() * (uint64_t)s_timebase_info.numer) / (uint64_t)s_timebase_info.denom);
}

static double getTime()
{
  // mach_absolute_time() returns billionth of seconds
  const double kOneBillion = 1000000000.0;
  return getTime_ns() / kOneBillion;
}

@interface KFEstimator ()
{
  double lastT;
  
  NSDate * lastDisturbance;
  int staticPts;
  
  kr::AttitudeMagCalib * magCalib;
}

@end

@implementation KFEstimator

- (id)init
{
  self = [super init];
  if (self != nil)
  {
    self.compassCalibrated = NO;
    
    _eskf = new kr::AttitudeESKF();
    _eskf->setEstimatesBias(true);
    _eskf->setUsesMagnetometer(false);
    _eskf->setGyroBiasThreshold(0.03f);
    _eskf->setFlipZAxis(true);
    
    kr::AttitudeESKF::VarSettings var;
    var.accel[0] = var.accel[1] = var.accel[2] = 1.0;
    var.gyro[0] = var.gyro[1] = var.gyro[2] = 1e-4;
    var.mag[0] = var.mag[1] = var.mag[2] = 0.1;
    
    _eskf->setVariances(var);
    
    magCalib = new kr::AttitudeMagCalib();
  }
  return self;
}

- (void)dealloc
{
  if (_eskf) {
    delete _eskf;
    _eskf=0;
  }
}

- (void)readAccel:(CMAcceleration)acceleration
            rates:(CMRotationRate)rotationRate
            field:(CMMagneticField)magneticField
{
  double T = getTime();
  double delta = std::max(std::min(T - lastT, 0.1), 0.01);
  lastT = T;
  
  auto ar = Vector3d(acceleration.x, acceleration.y, acceleration.z);
  auto gr = Vector3d(rotationRate.x, rotationRate.y, rotationRate.z);
  auto mr = Vector3d(magneticField.x*0.01, magneticField.y*0.01, magneticField.z*0.01);
  
  if (!self.gyroCalibrated)
  {
    if (std::abs(gr(0)) > 0.03f ||
        std::abs(gr(1)) > 0.03f ||
        std::abs(gr(2)) > 0.03f) {
      lastDisturbance = [NSDate date];
      NSLog(@"Disturbed!");
    }
    
    if (lastDisturbance.timeIntervalSinceNow < -0.5 || !lastDisturbance)
    {
      //  at 'rest', record point
      staticPts++;
    }
    
    //  most likely converged
    if (staticPts == 100) {
      self.gyroCalibrated = YES;
    }
  }
  /*else if (!self.compassCalibrated)
  {
    //NSLog(@"Adding sample: %f, %f, %f\n", mr[0], mr[1], mr[2]);
    
    magCalib->appendSample(_eskf->getQuat(), mr);
    
    if (magCalib->isReady()) {
      try {
        magCalib->calibrate(kr::AttitudeMagCalib::FullCalibration);
        self.compassCalibrated = YES;
      }
      catch(kr::AttitudeMagCalib::singular_hessian& e) {
        NSLog(@"Error: hessian was singular during calibration");
      }
    }
  }*/
  
  if (self.compassCalibrated) {

    //  subtract bias and apply scale
    /*Vector3d bias = magCalib->getBias();
    Vector3d scale = magCalib->getScale();
    
    for (int i=0; i < 3; i++) {
      mr[i] = (mr[i] - bias[i]) / scale[i];
    }
    
    _eskf->setMagneticReference( magCalib->getReference() );
    _eskf->setUsesMagnetometer(true);*/
  } else {
    _eskf->setUsesMagnetometer(false);
  }
  
  _eskf->predict(gr, delta);
  _eskf->update(ar,mr);
  
  auto Q = _eskf->getQuat();
  NSLog(@"%f, %f, %f, %f", Q.w(), Q.x(), Q.y(), Q.z());
}

@end
