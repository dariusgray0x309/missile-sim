use crate::missile::*;
pub struct Sim{
    pub msl  : Missile,
    pub tgt  : Vehicle,
    los      : LineOfSight,
    lsr      : LineOfSight,
    rng      : f64,
    r_dot    : f64,
    tm_data  : String
}

impl Sim{
    pub fn new(msl : Missile, tgt : Vehicle)->Self{
        Self{
            msl,
            tgt,
            los     : LineOfSight::default(),
            lsr     : LineOfSight::default(),
            rng     : 0.0,
            r_dot   : 0.0,
            tm_data : String::default()
        }
    }

    pub fn range(&self) -> f64{
        self.rng
    }

    pub fn range_rate(&self) -> f64{
        self.r_dot
    }

    pub fn init(&mut self, flight_path : f64, heading_error : f64) {

        // Converting from degrees to radians
        let heading_error_rad : f64 = 
            heading_error * (std::f64::consts::PI/180.0);

        let tgt_flight_path : f64 =
            flight_path * (std::f64::consts::PI/180.0);

        // Relative Quantites
        let del_x : f64 = self.tgt.pos.x - self.msl.pos.x;
        let del_y : f64 = self.tgt.pos.y - self.msl.pos.y;

        self.los.az = del_y.atan2(del_x);
        self.rng    = del_x.hypot(del_y);

        // Makes using the value a little less verbose
        let los : f64 = self.los.az;

        // Initial velocity magnitudes
        let vt          : f64 = self.tgt.vel.mag;
        let vm          : f64 = self.msl.vel.mag;
        let lead_angle  : f64 = ((vt/vm)*(tgt_flight_path + los).sin()).asin(); // sin^-1([vt/vm] *sin(BETA+LOS))

        // Initial velocity vectors (based on errors)
        self.msl.att.yaw = lead_angle + heading_error_rad + los;
        let vm_x : f64 = vm * (self.msl.att.yaw).cos(); // [m/s]
        let vm_y : f64 = vm * (self.msl.att.yaw).sin(); // [m/s]
        let vt_x : f64 = -vt * tgt_flight_path.cos();
        let vt_y : f64 = vt * tgt_flight_path.sin();

        // Final relative quantities
        let del_x_dot        : f64 = vt_x - vm_x;                                      // [m/s]
        let del_y_dot        : f64 = vt_y - vm_y;                                      // [m/s]
        let closing_velocity : f64 = -(del_x*del_x_dot + del_y*del_y_dot) / self.rng;  // [m/s]

        // Updating internal information
        self.msl.vel.x       = vm_x;
        self.msl.vel.y       = vm_y;
        self.msl.vel.mag     = vm;
        self.tgt.vel.x       = vt_x;
        self.tgt.vel.y       = vt_y;
        self.tgt.vel.mag     = vt;
        self.tgt.flight_path = tgt_flight_path;
        self.r_dot           = closing_velocity;

    }

    pub fn compute_slant_range(&mut self, del_x : f64, del_y : f64){
        self.rng = del_x.hypot(del_y);
    }

    pub fn compute_line_of_sight(&mut self, del_x : f64, del_y : f64){
        self.los.az = del_y.atan2(del_x);
    }

    pub fn compute_los_rate(&mut self, del_x : f64, del_y : f64, del_x_dot : f64, del_y_dot : f64){
        self.lsr.az = (del_x*del_y_dot - del_y*del_x_dot) / self.rng.powi(2);
    }

    pub fn compute_range_rate(&mut self, del_x : f64, del_y : f64, del_x_dot : f64, del_y_dot : f64){
        self.r_dot = -(del_x*del_x_dot + del_y*del_y_dot) / self.rng;
    }

    pub fn update_relative_kinematic_quantities(&mut self){
        // Update relative quantities
        let del_x       : f64 = self.tgt.pos.x - self.msl.pos.x;
        let del_y       : f64 = self.tgt.pos.y - self.msl.pos.y;
        let del_x_dot   : f64 = self.tgt.vel.x - self.msl.vel.x;
        let del_y_dot   : f64 = self.tgt.vel.y - self.msl.vel.y;

        // Calculate slant range
        self.compute_slant_range(del_x, del_y);

        // Calculate Line-of-sight
        self.compute_line_of_sight(del_x, del_y);

        // Calculate Line-of-sight rate
        self.compute_los_rate(del_x, del_y, del_x_dot, del_y_dot);

        // Calculate closing velocity
        self.compute_range_rate(del_x, del_y, del_x_dot, del_y_dot);
    }

    pub fn generate_pro_nav_commands(&mut self){
        // Calculate acceleration command for the missile
        self.msl.acc.mag = self.r_dot * self.lsr.az * self.msl.gain;

        // Convert to components
        self.msl.acc.x = -self.msl.acc.mag * (self.los.az).sin();
        self.msl.acc.y = self.msl.acc.mag * (self.los.az).cos();
    }

    pub fn update_target_velocity(&mut self){
        // Update target velocity
        self.tgt.vel.x = -self.tgt.vel.mag * (self.tgt.flight_path).cos();
        self.tgt.vel.y = self.tgt.vel.mag * (self.tgt.flight_path).sin();
    }

    pub fn update_target_flight_path(&mut self, dt : f64){
        // Update flight path angle rate of the target
        self.tgt.flight_path_rate = self.tgt.acc.mag / self.tgt.vel.mag;

        // Update flight path angle  of the target
        self.tgt.flight_path += self.tgt.flight_path_rate * dt;
    }

    pub fn set_telemetry(&mut self, t : f64){
        let pm_x = self.msl.pos.x;
        let pm_y: f64 = self.msl.pos.y;
        
        let pt_x: f64 = self.tgt.pos.x;
        let pt_y: f64 = self.tgt.pos.y;
        
        let vm_x: f64 = self.msl.vel.x;
        let vm_y: f64 = self.msl.vel.y;
        //
        let vt_x: f64 = self.tgt.vel.x;
        let vt_y: f64 = self.tgt.vel.y;
        //
        let los: f64 = self.los.az;
        let lsr: f64 = self.lsr.az;
        let slant_range: f64 = self.rng;
        let closing_velocity: f64 = self.r_dot;
        let am: f64 = self.msl.acc.mag;

        self.tm_data = format!("{:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}\n",
               t, pm_x, pm_y, pt_x, pt_y, vm_x, vm_y, vt_x, vt_y, los, lsr, slant_range, closing_velocity, am/9.81);
    }

    pub fn telemetry(&self) -> &String{
        &self.tm_data
    }

    pub fn print_telemetry(&self){
        println!("{}", self.tm_data);
    }

}