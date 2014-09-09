//
//  KFAppDelegate.m
//  kalman-ios
//
//  Created by Gareth Cross on 2013-10-01.
//  Copyright (c) 2013 gareth. Apache 2 License.
//

#import "KFAppDelegate.h"
#include "AttitudeESKF.hpp"

@interface KFAppDelegate ()
{
    BOOL sensorsAvailable;
}

- (void)didReadGyro:(CMGyroData *)data;
- (void)didReadAccelerometer:(CMAccelerometerData *)data;
- (void)didReadMagnetometer:(CMMagnetometerData *)data;

- (void)initializeKalmanFilter;
- (void)performKalmanUpdate;

@property (nonatomic) NSOperationQueue * queue;

@property (nonatomic) CMGyroData * gyroData;
@property (nonatomic) CMAccelerometerData * accelData;
@property (nonatomic) CMMagnetometerData * magnetometerData;
@end

@implementation KFAppDelegate

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
    //  create user interface
    self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
    self.window.backgroundColor = [UIColor whiteColor];
    
    //  load configuration screen
    self.calibrateVC = [[KFCalibrateViewController alloc] init];
    self.window.rootViewController = self.calibrateVC;
    
    [self.window makeKeyAndVisible];
    
    self.motionManager = [[CMMotionManager alloc] init];
    
    //  check for sensors
    sensorsAvailable = self.motionManager.isAccelerometerAvailable &&
                       self.motionManager.isGyroAvailable &&
                       self.motionManager.isMagnetometerAvailable;
    
    if (!sensorsAvailable)
    {
        self.calibrateVC.instructionLabel.text = @"This device does not have the sensors required to run this demo.";
        return YES;
    }
    else
    {
        self.calibrateVC.instructionLabel.text = @"Place the device on a level surface for a few seconds.";
    }
    
    //  set state at t=0
    [self initializeKalmanFilter];
    
    //  configure update intervals
    self.motionManager.accelerometerUpdateInterval = 0.01;
    self.motionManager.gyroUpdateInterval = 0.01;
    self.motionManager.magnetometerUpdateInterval = 0.01;
    
    //  queue for sensor readings
    self.queue = [NSOperationQueue mainQueue];
    KFAppDelegate __weak * wSelf = self;
    
    [self.motionManager startAccelerometerUpdatesToQueue:self.queue withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {
        if (accelerometerData && !error) {
            [wSelf didReadAccelerometer:accelerometerData];
        }
    }];
    
    [self.motionManager startGyroUpdatesToQueue:self.queue withHandler:^(CMGyroData *gyroData, NSError *error) {
        if (gyroData && !error) {
            [wSelf didReadGyro:gyroData];
        }
    }];
    
    [self.motionManager startMagnetometerUpdatesToQueue:self.queue withHandler:^(CMMagnetometerData *magnetometerData, NSError *error) {
        if (magnetometerData && !error) {
            [wSelf didReadMagnetometer:magnetometerData];
        }
    }];

    return YES;
}

- (void)didReadGyro:(CMGyroData *)data
{
    self.gyroData = data;
}

- (void)didReadMagnetometer:(CMMagnetometerData *)data
{
    self.magnetometerData = data;
}

- (void)didReadAccelerometer:(CMAccelerometerData *)data
{
    //  gyro or compass data not available yet, wait until next update
    if (!self.gyroData || !self.magnetometerData)
        return;
    self.accelData = data;
    
    //  run filter
    [self performKalmanUpdate];
    
    //  discard old data
    self.accelData = nil;
    self.gyroData = nil;
}

- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context
{
    if ([keyPath isEqualToString:@"gyroCalibrated"])
    {
        self.calibrateVC.instructionLabel.text = @"Rotate the device.";
    }
    else if ([keyPath isEqualToString:@"compassCalibrated"])
    {
        //  present the cube render view...
        self.renderVC = [VTViewController new];
        [self.window.rootViewController presentViewController:self.renderVC
                                                     animated:YES completion:NULL];
    }
}

- (void)initializeKalmanFilter
{
    self.estimator = [KFEstimator new];
    
    [self.estimator addObserver:self
                     forKeyPath:@"gyroCalibrated"
                        options:NSKeyValueObservingOptionNew
                        context:0];
    
    [self.estimator addObserver:self
                     forKeyPath:@"compassCalibrated"
                        options:NSKeyValueObservingOptionNew
                        context:0];
}

- (void)performKalmanUpdate
{
   [self.estimator readAccel:self.accelData.acceleration
                       rates:self.gyroData.rotationRate
                       field:self.magnetometerData.magneticField];
    
    if (self.estimator.compassCalibrated && self.estimator.gyroCalibrated)
    {
        //  log
        auto q = self.estimator.eskf->getQuat();
            
        //  send transform to render view
        auto R = q.matrix();
        auto M = GLKMatrix4Make(R(0,0), R(0,1), R(0,2), 0.0f,
                                R(1,0), R(1,1), R(1,2), 0.0f,
                                R(2,0), R(2,1), R(2,2), 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f);
        
        self.renderVC.cubeOrientation = M;
    }
}

#pragma mark - App State Change Stuff

- (void)applicationWillResignActive:(UIApplication *)application
{
}

- (void)applicationDidEnterBackground:(UIApplication *)application
{
}

- (void)applicationWillEnterForeground:(UIApplication *)application
{
}

- (void)applicationDidBecomeActive:(UIApplication *)application
{
}

- (void)applicationWillTerminate:(UIApplication *)application
{
}

@end
