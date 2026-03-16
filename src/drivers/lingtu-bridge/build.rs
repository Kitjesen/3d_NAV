fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .build_server(false) // 只需要 client
        .compile_protos(
            &["proto/han_dog_message/cms.proto"],
            &["proto"],
        )?;
    Ok(())
}
