varying highp vec3 colorValue;

void main()
{
    gl_FragColor = vec4(colorValue.r, colorValue.g, colorValue.b, 1.0);
}
