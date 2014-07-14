//
//  KFAppDelegate.h
//  kalman-ios
//
//  Created by Gareth Cross on 2013-10-01.
//  Copyright (c) 2013 gareth. Apache 2 License.
//

#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>

#import "KFCalibrateViewController.h"
#import "KFEstimator.h"
#import "VTViewController.h"

@interface KFAppDelegate : UIResponder <UIApplicationDelegate>
{}

@property (strong) CMMotionManager * motionManager;
@property (strong) KFEstimator * estimator;

@property (strong, nonatomic) KFCalibrateViewController * calibrateVC;
@property (strong, nonatomic) VTViewController * renderVC;
@property (strong, nonatomic) UIWindow *window;
@end
