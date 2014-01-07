attribute vec4 position;
attribute vec3 color;

varying highp vec3 colorValue;

uniform mat4 modelviewProjectionMatrix;

void main()
{
    colorValue = color;
    
    //  project vertices
    gl_Position = modelviewProjectionMatrix * position;
}
