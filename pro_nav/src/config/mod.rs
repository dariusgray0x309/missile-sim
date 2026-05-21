pub mod simulation;
use simulation::*;
use tonic::*;
use parameters_server::*;

#[derive(Default)]
pub struct SimParams{}

#[async_trait]
impl Parameters for SimParams{
    async fn get_configuration(
        &self,
        request: tonic::Request<EngagementSetupRequest>,
    ) -> std::result::Result<
        tonic::Response<EngagementSetupResponse>,
        tonic::Status,
    >{
        let msg = format!("Server: Received a request:\n{:?}", request);
        println!("{msg}");

        // Setup
        let missile_pos = Vector3d{
            x : 0.0,
            y : 10000.0,
            z : 0.0
        };

        let missile_vel = Vector3d{
            x : 1732.05,
            y : 1732.05,
            z : 1732.05
        };

        let missile_acc = Vector3d{
            x : 0.0,
            y : 0.0,
            z : 0.0
        };

        let missile_att = Attitude{
            roll  : 0.0,
            pitch : 0.0,
            yaw   : 0.0
        };

        let missile_kinematics = Kinematics{
            pos : Some(missile_pos),
            vel : Some(missile_vel),
            acc : Some(missile_acc),
            att : Some(missile_att)
        };

        let missile_uncertainty = Uncertainty::default();
        
        //missile_uncertainty.att_stddev.unwrap().yaw = -20.0;

        let pursuer = Pursuer{
            states      : Some(missile_kinematics),
            nav_gain    : 4.0,
            uncertainty : Some(missile_uncertainty)
        };

        let target_pos = Vector3d{
            x : 40000.0,
            y : 10000.0,
            z : 0.0
        };

        let target_vel = Vector3d{
            x : 577.35,
            y : 577.35,
            z : 577.35
        };

        let target_acc = Vector3d{
            x : 0.0,
            y : 0.0,
            z : 0.0
        };

        let target_att = Attitude{
            roll  : 0.0,
            pitch : 0.0,
            yaw   : 0.0
        };

        let target_kinematics = Kinematics{
            pos : Some(target_pos),
            vel : Some(target_vel),
            acc : Some(target_acc),
            att : Some(target_att)
        };

        let target = Target{
            states      : Some(target_kinematics),
            flight_path : 0.0
        };
        //
        
        let repsonse = EngagementSetupResponse{
            pursuer : Some(pursuer),
            target  : Some(target),
            success : true
        };

        Ok(tonic::Response::new(repsonse))
        
    }
}
