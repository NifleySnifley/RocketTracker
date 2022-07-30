use eframe::egui::mutex::Mutex;
use egui_glow;
use egui_glow::glow;

use image::io::Reader as ImageReader;
use image::GenericImageView;
use std::fs;
use std::io::Cursor;

pub struct CompassWidget {
    program: glow::Program,
    vertex_array: glow::VertexArray,
    pointer_texture: glow::NativeTexture,
    dial_texture: glow::NativeTexture,
}

#[allow(unsafe_code)] // we need unsafe code to use glow
impl CompassWidget {
    pub fn load_texture(gl: &glow::Context, path: &str) -> glow::NativeTexture {
        unsafe {
            use glow::HasContext as _;
            let img = ImageReader::open(path)
                .unwrap()
                .decode()
                .unwrap()
                .into_rgba8();
            let img_data = img.clone().into_flat_samples().samples;

            let tex = gl.create_texture().unwrap();
            gl.bind_texture(glow::TEXTURE_2D, Some(tex));
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR_MIPMAP_LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA as i32,
                img.width() as i32,
                img.height() as i32,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                Some(img_data.as_slice()),
            );
            gl.generate_mipmap(glow::TEXTURE_2D);

            tex
        }
    }

    pub fn new(gl: &glow::Context) -> Self {
        use glow::HasContext as _;

        let shader_version = "#version 300 es";

        unsafe {
            let tex = CompassWidget::load_texture(gl, "./res/pointer.png");
            let dial_tex = CompassWidget::load_texture(gl, "./res/dial.png");

            let program = gl.create_program().expect("Cannot create program");

            let (vert_file, frag_file) = (
                fs::read_to_string("res/compass.vert").expect("Shader FS error"),
                fs::read_to_string("res/compass.frag").expect("Shader FS error"),
            );

            let (vertex_shader_source, fragment_shader_source) =
                (vert_file.as_str(), frag_file.as_str());

            let shader_sources = [
                (glow::VERTEX_SHADER, vertex_shader_source),
                (glow::FRAGMENT_SHADER, fragment_shader_source),
            ];

            let shaders: Vec<_> = shader_sources
                .iter()
                .map(|(shader_type, shader_source)| {
                    let shader = gl
                        .create_shader(*shader_type)
                        .expect("Cannot create shader");
                    gl.shader_source(shader, &format!("{}\n{}", shader_version, shader_source));
                    gl.compile_shader(shader);
                    if !gl.get_shader_compile_status(shader) {
                        panic!("{}", gl.get_shader_info_log(shader));
                    }
                    gl.attach_shader(program, shader);
                    shader
                })
                .collect();

            gl.link_program(program);
            if !gl.get_program_link_status(program) {
                panic!("{}", gl.get_program_info_log(program));
            }

            for shader in shaders {
                gl.detach_shader(program, shader);
                gl.delete_shader(shader);
            }

            let vertex_array = gl
                .create_vertex_array()
                .expect("Cannot create vertex array");

            Self {
                program,
                vertex_array,
                pointer_texture: tex,
                dial_texture: dial_tex,
            }
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.delete_program(self.program);
            gl.delete_vertex_array(self.vertex_array);
        }
    }

    pub fn paint(&self, gl: &glow::Context, compass_heading: f64) {
        use glow::HasContext as _;
        unsafe {
            // Draw dial
            gl.use_program(Some(self.program));
            gl.uniform_1_f32(
                gl.get_uniform_location(self.program, "heading").as_ref(),
                0f32,
            );
            gl.bind_texture(glow::TEXTURE_2D, Some(self.dial_texture));
            gl.bind_vertex_array(Some(self.vertex_array));
            gl.draw_arrays(glow::TRIANGLE_STRIP, 0, 4);

            // Draw pointer
            gl.uniform_1_f32(
                gl.get_uniform_location(self.program, "heading").as_ref(),
                compass_heading as f32,
            );
            gl.bind_texture(glow::TEXTURE_2D, Some(self.pointer_texture));
            gl.bind_vertex_array(Some(self.vertex_array));
            gl.draw_arrays(glow::TRIANGLE_STRIP, 0, 4);
        }
    }
}
