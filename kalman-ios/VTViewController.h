//
//  VTViewController.h
//  VideoTest
//
//  Created by Gareth Cross on 2012-09-29.
//  Copyright (c) 2012 deus-ostrum. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

@interface VTViewController : GLKViewController
{
}

@property (readwrite, nonatomic) GLKMatrix4 cubeOrientation;
@end
