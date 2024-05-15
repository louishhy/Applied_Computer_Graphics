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
        // === Code added from here ===
        // First let us denote a vector pp0 = (x0, y0, z0) - org. p is the point on the armadillo, p = (x0, y0, z0).
        vec3 pp0 = vec3(x0, y0, z0) - org;
        // Let us project pp0 onto the normal vector nrm.
        vec3 pp0_proj = dot(pp0, nrm) * nrm;
        // The reflected point is given by p - 2 * pp0_proj.
        vec3 p_reflected = vec3(x0, y0, z0) - 2.0 * pp0_proj;
        // Translate the rendered object outwards by trans_len,
        // so that it is contained in the frustum.
        // P.S. A more general method I propose is that
        // we get the point w/ the largest projected z.
        // Then, we calculate the maximum overflow amount and 
        // can translate toward the screen by that amount.
        // However, we are only required to modify the .vert, 
        // so I will just use a tuned fixed value.
        float trans_len = 0.5;
        p_reflected.z = p_reflected.z + trans_len;
        // Supress the reflection when reflected z is larger than the original z
        // AND the point behind the original point in the foreground.
        // (This is induced by the translation-out operation)
        // This handles the occlusion.
        if (p_reflected.z >= z0 && p_reflected.x == x0 && p_reflected.y == y0){
            // Render the point behind the original object, so that it is occluded
            float epsilon = 0.001;
            x0 = p_reflected.x;
            y0 = p_reflected.y;
            z0 = z0 - epsilon;
        }
        else {
            // Render as usual.
            x0 = p_reflected.x;
            y0 = p_reflected.y;
            z0 = p_reflected.z;
        }
    }
    // do not edit below

    // "gl_Position" is the *output* vertex coordinate in the
    // "canonical view volume (i.e.. [-1,+1]^3)" pass to the rasterizer.
    gl_Position = vec4(x0, y0, -z0, 1);// opengl actually draw a pixel with *maximum* depth. so invert z
}
