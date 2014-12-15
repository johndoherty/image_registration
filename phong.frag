// phong.frag

varying float distToCamera;    // fragment position in model space

void main() {
	float maxDist = 3500.0;
	float color = 1.0 - (distToCamera / maxDist);
	if (color < 0.0) color = 0.0;
	if (color > 1.0) color = 1.0;
    gl_FragColor = vec4(color, color, color, 1.0);
    //gl_FragColor = gl_Color;
}
