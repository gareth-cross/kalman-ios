//
//  CCUDPClient.m
//  capture-client
//
//  Created by Gareth Cross on 2013-10-05.
//  Copyright (c) 2013 gareth. All rights reserved.
//

#import "CCUDPClient.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

@interface CCUDPClient ()
{
}

@property (readwrite, assign) int descriptor;
@end

@implementation CCUDPClient

- (id)init
{
    self = [super init];
    if (self != nil)
    {
        self.descriptor = -1;
    }
    return self;
}

- (BOOL)connect:(NSString *)address port:(unsigned short)port error:(NSError **)error
{
    [self stop];
    NSString * portStr = [NSString stringWithFormat:@"%u", port];
    
    //  open a socket
    self.descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (self.descriptor < 0)
    {
        if (error) {
            *error = [NSError errorWithDomain:@"CCUDPClient"
                                         code:errno
                                     userInfo:@{NSLocalizedDescriptionKey :
                                                    [NSString stringWithFormat:@"%s", strerror(errno)]}];
        }
        
        return NO;
    }
    
    //  do hostname lookup
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    
    if (getaddrinfo([address cStringUsingEncoding:NSASCIIStringEncoding], [portStr cStringUsingEncoding:NSASCIIStringEncoding], &hints, &res))
    {
        if (error) {
            *error = [NSError errorWithDomain:@"CCUDPClient"
                                         code:errno
                                     userInfo:@{NSLocalizedDescriptionKey : @"getaddrinfo failed"}];
        }
        
        [self stop];
        return NO;
    }
    
    BOOL connected = YES;
    if (connect(self.descriptor, res->ai_addr, res->ai_addrlen))
    {
        if (error) {
            *error = [NSError errorWithDomain:@"CCUDPClient"
                                         code:errno
                                     userInfo:@{NSLocalizedDescriptionKey :
                                                    [NSString stringWithFormat:@"%s", strerror(errno)]}];
        }
        
        connected = NO;
    }
    
    //  cleanup names...
    freeaddrinfo(res);
    
    return connected;
}

- (void)stop
{
    if (self.descriptor >= 0) {
        close(self.descriptor);
        self.descriptor = -1;
    }
}

- (BOOL)sendData:(NSData *)data
{
    if (self.descriptor >= 0)
    {
        ssize_t s = send(self.descriptor, data.bytes, data.length, 0);
        if (s != data.length) {
            return NO;
        }
    }
    
    return YES;
}

@end
