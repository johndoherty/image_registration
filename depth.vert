// default.vert

// This 'varying' vertex output can be read as an input
// by a fragment shader that makes the same declaration.
varying float distToCamera;

uniform mat4 model_view;
uniform mat4 projection;

void main() {
    
    //vec4 cs_position = cameraPose * gl_ModelViewMatrix * gl_Vertex;
    //vec4 cs_position = gl_ModelViewMatrix * cameraPose * gl_Vertex;
    vec4 cs_position = model_view * gl_Vertex;
    distToCamera = -cs_position.z;
    gl_Position = projection * cs_position;
    gl_FrontColor = gl_Color;
    
    // Render the shape using modified position.
    //gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix *  vec4(modelPos,1);
}
