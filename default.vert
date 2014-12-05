// default.vert

/*
  This simple GLSL vertex shader does exactly what 
  OpenGL would do -- It transforms the vertex positions
  using the default OpenGL transformation. It also passes
  through the texture coordinate, normal coordinate, and some
  other good stuff so we can use them in the fragment shader.
*/


// This 'varying' vertex output can be read as an input
// by a fragment shader that makes the same declaration.
varying float distToCamera;

uniform mat4 cameraPose;

void main() {
    
    //vec4 cs_position = cameraPose * gl_ModelViewMatrix * gl_Vertex;
    vec4 cs_position = gl_ModelViewMatrix * cameraPose * gl_Vertex;
    distToCamera = -cs_position.z;
    gl_Position = gl_ProjectionMatrix * cs_position;
    gl_FrontColor = gl_Color;
    
    // Render the shape using modified position.
    //gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix *  vec4(modelPos,1);
}
