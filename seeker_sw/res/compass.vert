#define PI 3.1415926535
const vec2 verts[4] = vec2[4](
    vec2(-1.0,  1.0 ),
    vec2(-1.0, -1.0 ),
    vec2( 1.0,  1.0 ),
    vec2( 1.0, -1.0 )
);

const vec2 uvs[4] = vec2[4](
    vec2( 1.0,  0.0 ),
    vec2( 1.0,  1.0 ),
    vec2( 0.0,  0.0 ),
    vec2( 0.0,  1.0 )
);

out vec2 v_uv;
uniform float heading;

void main() {
    v_uv = uvs[gl_VertexID];
    float h = -(PI - heading);
    mat2 mat = mat2(
        cos(h), sin(h),
        sin(h), -cos(h)
    ) * 1.15;
    gl_Position = vec4(verts[gl_VertexID], 0.0, 1.0);
    gl_Position.xy *= mat;
}