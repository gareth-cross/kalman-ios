//
//  VTViewController.m
//  VideoTest
//
//  Created by Gareth Cross on 2012-09-29.
//  Copyright (c) 2012 deus-ostrum. All rights reserved.
//

#import "VTViewController.h"
#include "CubeData.h"

@interface VTViewController ()
{
    /*
     *  Rendering Data
     */
    GLuint shaderProgram;
    int mdlUniform;
}

@property (nonatomic) EAGLContext *context;
@end

@implementation VTViewController

- (id)init
{
    self = [super initWithNibName:@"VTViewController" bundle:nil];
    if (self != nil) {
        
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    
    GLKView * view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    view.drawableColorFormat = GLKViewDrawableColorFormatRGBA8888;
    
    [EAGLContext setCurrentContext:self.context];
    
    NSString *vertexShaderSource = [NSString stringWithContentsOfFile:[[NSBundle mainBundle] pathForResource:@"Vertex" ofType:@"vsh"] encoding:NSUTF8StringEncoding error:nil];
    const char *vertexShaderSourceCString = [vertexShaderSource cStringUsingEncoding:NSUTF8StringEncoding];
    
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSourceCString, NULL);
    glCompileShader(vertexShader);
    
    GLint logLength=0;
    glGetShaderiv(vertexShader, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetShaderInfoLog(vertexShader, logLength, &logLength, log);
        NSLog(@"Shader compile log:\n%s", log);
        free(log);
    }
    
    NSString *fragmentShaderSource = [NSString stringWithContentsOfFile:[[NSBundle mainBundle] pathForResource:@"Fragment" ofType:@"fsh"] encoding:NSUTF8StringEncoding error:nil];
    const char *fragmentShaderSourceCString = [fragmentShaderSource cStringUsingEncoding:NSUTF8StringEncoding];
    
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSourceCString, NULL);
    glCompileShader(fragmentShader);
    
    glGetShaderiv(fragmentShader, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetShaderInfoLog(fragmentShader, logLength, &logLength, log);
        NSLog(@"Shader compile log:\n%s", log);
        free(log);
    }
    
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    
    glBindAttribLocation(shaderProgram, GLKVertexAttribPosition, "position");
    glBindAttribLocation(shaderProgram, GLKVertexAttribColor, "color");
    
    glLinkProgram(shaderProgram);
    
    glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0)
    {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetProgramInfoLog(shaderProgram, logLength, &logLength, log);
        NSLog(@"Program link log:\n%s", log);
        free(log);
    }
    
    mdlUniform = glGetUniformLocation(shaderProgram, "modelviewProjectionMatrix");
}

- (void)viewDidAppear:(BOOL)animated
{
}

- (void)viewWillDisappear:(BOOL)animated
{
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glViewport(0, 0,
               (GLint)(rect.size.width * [UIScreen mainScreen].scale),  //  multiply by scale for retina displays
               (GLint)(rect.size.height * [UIScreen mainScreen].scale));
    
    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
    //  configure state engine
    glShadeModel(GL_SMOOTH);
	
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
    
    GLKMatrix4 projectionMatrix = GLKMatrix4MakePerspective(M_PI / 2.0f, rect.size.width / rect.size.height, 0.1f, 10.0f);
    GLKMatrix4 mdlMtx = GLKMatrix4Identity;
    
    //  translate and rotate cube
    mdlMtx = GLKMatrix4Translate(mdlMtx, 0.0f, 0.0f, -4.0f);
    mdlMtx = GLKMatrix4Multiply(mdlMtx, self.cubeOrientation);
    
    //  project
    mdlMtx = GLKMatrix4Multiply(projectionMatrix, mdlMtx);
    
    //  Set variables in GLSL shaders
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(mdlUniform, 1, 0, mdlMtx.m);
    
    //  bind arrays of a unit cube
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 0, cube_vertices);
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    
    glVertexAttribPointer(GLKVertexAttribColor, 3, GL_FLOAT, GL_FALSE, 0, cube_colors);
    glEnableVertexAttribArray(GLKVertexAttribColor);
    
    //  render...
    glDrawArrays(GL_TRIANGLES, 0, 36);
    
    //  done with attrib arrays
    glDisableVertexAttribArray(GLKVertexAttribColor);
    glDisableVertexAttribArray(GLKVertexAttribPosition);
    
    glFlush();
}

@end
