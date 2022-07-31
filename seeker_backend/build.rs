use std::io::Result;

fn main() -> Result<()> {
    // TODO use FS to search folder and compile all protos
    prost_build::compile_protos(
        &[
            "../protobufs/configuration.proto",
            "../protobufs/messages.proto",
        ],
        &["../protobufs/"],
    )?;
    Ok(())
}
