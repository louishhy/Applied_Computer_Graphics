#version 120

// see the GLSL 1.2 specification:
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.1.20.pdf

uniform bool is_reflection; // variable of the program
varying vec3 normal; // normal vector pass to the rasterizer and fragment shader

void main()
{
    normal = vec3(gl_Normal);// set normal and pass it to fragment shader

    // "gl_Vertex" is the *input* vertex coordinate of triangle.
    // "gl_Vertex" has type of "vec4", which is homogeneious coordinate
    float x0 = gl_Vertex.x/gl_Vertex.w;// x-coord
    float y0 = gl_Vertex.y/gl_Vertex.w;// y-coord
    float z0 = gl_Vertex.z/gl_Vertex.w;// z-coord
    if (is_reflection) {
        vec3 nrm = normalize(vec3(0.4, 0.0, 1.0)); // normal of the mirror
        vec3 org = vec3(-0.3, 0.0, -0.5); // point on the mirror
        // wite code to change the input position (x0,y0,z0).
        // the transformed position (x0, y0, z0) should be drawn as the mirror reflection.
        //
        // make sure the occlusion is correctly computed.
        // the mirror is behind the armadillo, so the reflected image should be behind the armadillo.
        // furthermore, make sure the occlusion is correctly computed for the reflected image.
        // First let us denote a vector pp0 = (x0, y0, z0) - org. p is the point on the armadillo, p = (x0, y0, z0).
        vec3 pp0 = vec3(x0, y0, z0) - org;
        // Let us project pp0 onto the normal vector nrm.
        vec3 pp0_proj = dot(pp0, nrm) * nrm;
        // The reflected point is given by p - 2 * pp0_proj.
        vec3 p_reflected = vec3(x0, y0, z0) - 2.0 * pp0_proj;
        x0 = p_reflected.x;
        y0 = p_reflected.y;
        z0 = p_reflected.z;
    }
    // do not edit below

    // "gl_Position" is the *output* vertex coordinate in the
    // "canonical view volume (i.e.. [-1,+1]^3)" pass to the rasterizer.
    gl_Position = vec4(x0, y0, -z0, 1);// opengl actually draw a pixel with *maximum* depth. so invert z
}
