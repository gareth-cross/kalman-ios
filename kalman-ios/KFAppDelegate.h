//
//  KFAppDelegate.h
//  kalman-ios
//
//  Created by Gareth Cross on 2013-10-01.
//  Copyright (c) 2013 gareth. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>

#import "CCUDPClient.h"
#import "KFCalibrateViewController.h"
#import "KFEstimator.h"
#import "VTViewController.h"

@interface KFAppDelegate : UIResponder <UIApplicationDelegate>
{}

@property (strong) CMMotionManager * motionManager;
@property (strong) KFEstimator * estimator;
@property (strong) CCUDPClient * client;

@property (strong, nonatomic) KFCalibrateViewController * calibrateVC;
@property (strong, nonatomic) VTViewController * renderVC;
@property (strong, nonatomic) UIWindow *window;
@end
