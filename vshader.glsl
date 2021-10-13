#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec4 a_position;

varying vec4 v_position;

//! [0]
void main()
{
    // Calculate vertex position in screen space
    gl_Position = mvp_matrix * a_position;

    // Pass vertex position to fragment shader
    // Value will be automatically interpolated to fragments inside polygon faces
    v_position = a_position;
}
//! [0]
