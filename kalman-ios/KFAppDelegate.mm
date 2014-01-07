//
//  KFAppDelegate.m
//  kalman-ios
//
//  Created by Gareth Cross on 2013-10-01.
//  Copyright (c) 2013 gareth. All rights reserved.
//

#import "KFAppDelegate.h"
#include "AttitudeESKF.hpp"

template <typename T>
void pack_data(char * dest, const T * data, int count, unsigned int &pos)
{
	memcpy(&dest[pos], data, sizeof(T)*count);
	pos += sizeof(T) * count;
}

struct KalmanState
{
    KalmanState() {}
    
    float ang[3];
    float qvals[4];
};

@interface KFAppDelegate ()
{
    KalmanState * _s;
    
    BOOL sensorsAvailable;
    AttitudeESKF * eskf;
}

- (void)didReadGyro:(CMGyroData *)data;
- (void)didReadAccelerometer:(CMAccelerometerData *)data;
- (void)didReadMagnetometer:(CMMagnetometerData *)data;

- (void)initializeKalmanFilter;
- (void)performKalmanUpdate;

- (void)transmitState;

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
    
    //  udp socket to matlab
    self.client = [[CCUDPClient alloc] init];
    
    NSError * err;
    if (![self.client connect:@"10.0.1.130" port:18000 error:&err]) {
        NSLog(@"Failed to open socket: %@", err.localizedDescription);
    }

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
    
    //  send state to server
    [self transmitState];
    
    //  discard old data
    self.accelData = nil;
    self.gyroData = nil;
}

- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context
{
    if ([keyPath isEqualToString:@"gyroCalibrated"])
    {
        self.calibrateVC.instructionLabel.text = @"Rotate the device in all directions.";
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
    _s = new KalmanState();
    
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
        auto q = self.estimator.eskf->getState();
        auto g = self.estimator.eskf->getAPred();
        
        auto a = self.accelData.acceleration;
        auto w = self.gyroData.rotationRate;
        
        auto mp = self.estimator.eskf->getMPred();
        auto m = self.estimator.eskf->getMMeas();
        
        //  uncomment to log state...
        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
               q.a(), q.b(), q.c(), q.d(),
               -a.x, -a.y, -a.z,
               g(0), g(1), g(2),
               -w.x, -w.y, -w.z,
               m(0), m(1), m(2),
               mp(0), mp(1), mp(2));
   
        _s->qvals[0] = q.a();
        _s->qvals[1] = q.b();
        _s->qvals[2] = q.c();
        _s->qvals[3] = q.d();
        
        //  calculate un-filtered angles
        float ay = -a.y;
        if (ay < -1.0f) {
            ay = -1.0f;
        } else if (ay > 1.0f) {
            ay = 1.0f;
        }
        _s->ang[1] = std::atan2(-a.x, -a.z);
        _s->ang[0] = std::asin(-ay);
        _s->ang[2] = std::atan2(m(1), m(0));    //  hack: using the filtered cos/theta to tilt-compensate here
        
        //  send transform to render view
        auto R = q.to_matrix();
        
        GLKMatrix4 trans = GLKMatrix4Identity;
        auto M = GLKMatrix4MakeAndTranspose(R(0,0), R(0,1), R(0,2), 0.0f,
                                            R(1,0), R(1,1), R(1,2), 0.0f,
                                            R(2,0), R(2,1), R(2,2), 0.0f,
                                            0.0f, 0.0f, 0.0f, 1.0f);
        
        trans = GLKMatrix4Multiply(trans, M);
        
        self.renderVC.cubeOrientation = trans;
    }
}

- (void)transmitState
{
    char * payload = (char *)malloc(4096);
    unsigned int len=0;
    
    uint32_t marker = 0x6B756D61;
    
    pack_data(payload, &marker, 1, len);
    pack_data(payload, _s->ang, 3, len);    //  empty in this version...
    pack_data(payload, _s->qvals, 4, len);
    
    [self.client sendData:[NSData dataWithBytes:payload
                                         length:len]];

    free(payload);
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
