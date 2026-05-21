use std::env;
use std::fs::{OpenOptions,metadata};
use std::io::Write;
use std::path::PathBuf;

use crate::missile::*;
use crate::config::simulation::*;

pub fn run_sim(params : EngagementSetupResponse) -> Result<(), Box<dyn std::error::Error>>{

    // Get the current directory
    let current_dir: PathBuf = env::current_dir()?;

    let file_path = current_dir.join("sim_output/example_01.txt");

    // Check if the directory exists and create it if it doesn't
    if let Some(parent) = file_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create directories");
        }
    }

    // Labels for the columns (written as the first row if the file is created)
    let labels = "time   m_x   m_y   t_x   t_y   m_vx   m_vy   t_vx   t_vy   LOS   LSR   range   cvel   m_accel\n";

    // Check if the file exists (clone the PathBuf here)
    // Cloning: file_path.clone() creates a new PathBuf instance with the same value, 
    // so you retain ownership of the original file_path.
    let file_exists = metadata(file_path.clone()).is_ok();

    // Attempt to open or create the file
    let mut file = match OpenOptions::new()
        .write(true) // allow writing
        .create(true) // create the file if it doesn't exist
        .append(true) // append the file if it exists
        .open(file_path)
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Error opening file: {}", e);
            Err(e)
            
        }?
    };

    // Write labels only if the file is being created
    if !file_exists {
        file.write_all(labels.as_bytes()).expect("Failed to write labels to file");
    }

    // -- 2D Tactical Missile -> Target Engagement --

    // Time related parameters
    let mut t    : f64 = 0.0;   // time
    let dt       : f64 = 0.01;  // time step
    let max_time : f64 = 100.0; // max simulation time in seconds

    let mut msl : Missile = Missile::new();
    let mut tgt : Vehicle = Vehicle::new();

    // Use inputs from gRPC here:
    let missile_px = params.pursuer.unwrap().states.unwrap().pos.unwrap().x;
    let missile_py = params.pursuer.unwrap().states.unwrap().pos.unwrap().y;
    let missile_pz = params.pursuer.unwrap().states.unwrap().pos.unwrap().z;

    let missile_vx = params.pursuer.unwrap().states.unwrap().vel.unwrap().x;
    let missile_vy = params.pursuer.unwrap().states.unwrap().vel.unwrap().y;
    let missile_vz = params.pursuer.unwrap().states.unwrap().vel.unwrap().z;

    let mut missile_v_mag = Velocity::default();
    missile_v_mag.set_vals(missile_vx, missile_vy, missile_vz);
        
    msl.pos.set_vals(missile_px, missile_py, missile_pz); // [m]
    msl.vel.set_mag(missile_v_mag.compute_magnitude());          // [m/s]
    msl.set_nav_gain(params.pursuer.unwrap().nav_gain);

    let target_px = params.target.unwrap().states.unwrap().pos.unwrap().x;
    let target_py = params.target.unwrap().states.unwrap().pos.unwrap().y;
    let target_pz = params.target.unwrap().states.unwrap().pos.unwrap().z;

    let target_vx = params.target.unwrap().states.unwrap().vel.unwrap().x;
    let target_vy = params.target.unwrap().states.unwrap().vel.unwrap().y;
    let target_vz = params.target.unwrap().states.unwrap().vel.unwrap().z;

    let mut target_v_mag = Velocity::default();
    target_v_mag.set_vals(target_vx, target_vy, target_vz);

    tgt.pos.set_vals(target_px, target_py, target_pz);
    tgt.vel.set_mag(target_v_mag.compute_magnitude()); // [m/s]

    let mut missile_sim = engagement::Sim::new(msl, tgt);
    missile_sim.init(params.target.unwrap().flight_path, -20.0);
    
    // Store the updated data
    missile_sim.set_telemetry(t);

    // Write to text file
    file.write_all(missile_sim.telemetry().as_bytes()).expect("Failed to write data to file");

    // Beginning Simulation:
    println!("\nBeginning Missile Simulation:");

    while missile_sim.range_rate() >= 0.0 && t <= max_time {

        // Propagate the missile
        missile_sim.msl.vel.euler_integrate(&missile_sim.msl.acc, dt);
        missile_sim.msl.pos.euler_integrate(&missile_sim.msl.vel, dt);

        // Propagate the target
        missile_sim.tgt.pos.euler_integrate(&missile_sim.tgt.vel, dt);

        // Calculate range, range rate, LOS, & LSR
        missile_sim.update_relative_kinematic_quantities();

        // Calculate acceleration command for the missile
        missile_sim.generate_pro_nav_commands();

        // Update target velocity
        missile_sim.update_target_velocity();

        // Update flight path angle rate of the target
        missile_sim.update_target_flight_path(dt);

        // Store the updated data
        missile_sim.set_telemetry(t);

        // Write to text file
        file.write_all(missile_sim.telemetry().as_bytes()).expect("Failed to write data to file");

        // Increment time
        t += dt;

    }

    // For debugging
    println!("Complete!\n");

    Ok(())

}