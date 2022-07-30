precision mediump float;
in vec2 v_uv;
out vec4 out_color;
uniform sampler2D texture;

void main() {
    out_color = texture2D(texture, v_uv);
}