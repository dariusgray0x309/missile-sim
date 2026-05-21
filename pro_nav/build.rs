fn main()-> Result<(), Box<dyn std::error::Error>>{
    let proto_path = "proto/mission.proto";
    tonic_build::compile_protos(proto_path)?;
    Ok(())
}