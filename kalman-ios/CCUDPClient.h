//
//  CCUDPClient.h
//  capture-client
//
//  Created by Gareth Cross on 2013-10-05.
//  Copyright (c) 2013 gareth. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface CCUDPClient : NSObject

- (id)init;

- (BOOL)connect:(NSString *)address port:(unsigned short)port error:(NSError **)error;

- (void)stop;

- (BOOL)sendData:(NSData *)data;

@end
