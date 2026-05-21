use pro_nav::config::SimParams;
use pro_nav::config::simulation::parameters_server::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>>{
    println!("Creating the server");

    let endpoint = "0.0.0.0:5555".parse()?;

    // Create the server
    let app = ParametersServer::new(SimParams::default());

    tonic::transport::Server::builder()
        .add_service(app)
        .serve(endpoint)
        .await?;

    Ok(())
}
