use pro_nav::config::simulation::parameters_client::*;
use pro_nav::config::simulation::EngagementSetupRequest;
use pro_nav::sim::run_sim;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>>{

    let endpoint = "http://0.0.0.0:5555";

    // Create the client and connect to the address of the server 
    let mut client = ParametersClient::connect(endpoint).await?;

    // Create the request to the server
    let output_msg = String::from("Provide simulation parameters");
    let request = tonic::Request::new(
        EngagementSetupRequest{
            message : output_msg
        }
    );

    println!("Client: Successfully connected to the server at {}\n", endpoint);

    // Call the function
    let response = client.get_configuration(request).await?;

    println!("RESPONSE = {:?}", response);

    // Now that we got the reponse, run the simulation
    run_sim(response.into_inner())

} 
