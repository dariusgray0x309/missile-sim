pub mod engagement;

#[derive(Default)]
pub struct Position{
    x   : f64,
    y   : f64,
    z   : f64,
    mag : f64
}

impl Position{
    pub fn set_vals (&mut self, x : f64, y : f64, z : f64){
        self.x = x;
        self.y = y;
        self.z = z;
    }

    pub fn set_mag(&mut self, mag : f64){
        self.mag = mag;
    }
}

#[derive(Default)]
pub struct Velocity{
    x   : f64,
    y   : f64,
    z   : f64,
    mag : f64
}

impl Velocity{
    pub fn set_vals (&mut self, x : f64, y : f64, z : f64){
        self.x = x;
        self.y = y;
        self.z = z;
    }

    pub fn set_mag(&mut self, mag : f64){
        self.mag = mag;
    }
}

#[derive(Default)]
pub struct Acceleration{
    x   : f64,
    y   : f64,
    z   : f64,
    mag : f64
}

impl Acceleration{
    pub fn set_vals (&mut self, x : f64, y : f64, z : f64){
        self.x = x;
        self.y = y;
        self.z = z;
    }

    pub fn set_mag(&mut self, mag : f64){
        self.mag = mag;
    }
}

#[allow(dead_code)]
#[derive(Default)]
pub struct Attitude{
    roll  : f64,
    pitch : f64,
    yaw   : f64
}

#[allow(dead_code)]
#[derive(Default)]
struct LineOfSight{
    az : f64,
    el : f64
}

pub trait Magnitude{
    fn compute_magnitude(&self) -> f64;
}

impl Magnitude for Position{
    fn compute_magnitude(&self) -> f64{
        let x : f64 = self.x;
        let y : f64 = self.y;
        let z : f64 = self.z;
        
        (x*x + y*y + z*z).sqrt()
    }
}

impl Magnitude for Velocity{
    fn compute_magnitude(&self) -> f64{
        let x : f64 = self.x;
        let y : f64 = self.y;
        let z : f64 = self.z;
        
        (x*x + y*y + z*z).sqrt()
    }
}

impl Magnitude for Acceleration{
    fn compute_magnitude(&self) -> f64{
        let x : f64 = self.x;
        let y : f64 = self.y;
        let z : f64 = self.z;
        
        (x*x + y*y + z*z).sqrt()
    }
}

pub trait Propagate<T>{
    fn euler_integrate(&mut self, derivative : &T, dt : f64);
}

impl Propagate<Velocity> for Position{
    fn euler_integrate(&mut self, derivative : &Velocity, dt : f64) {
        self.x += derivative.x * dt;
        self.y += derivative.y * dt;
        self.z += derivative.z * dt;
    }
}

impl Propagate<Acceleration> for Velocity{
    fn euler_integrate(&mut self, derivative : &Acceleration, dt : f64) {
        self.x += derivative.x * dt;
        self.y += derivative.y * dt;
        self.z += derivative.z * dt;
    }
}

pub struct Missile{
    pub pos              : Position,
    pub vel              : Velocity,
    pub acc              : Acceleration,
    pub att              : Attitude,
    pub gain             : f64,
    pub tof              : f64,
    pub flight_path      : f64,
    pub flight_path_rate : f64
}

impl Missile{
    pub fn new() ->Self{
        Self { 
            pos              : Position::default(),
            vel              : Velocity::default(), 
            acc              : Acceleration::default(), 
            att              : Attitude::default(),
            tof              : 0.0,
            gain             : 0.0, 
            flight_path      : 0.0, 
            flight_path_rate : 0.0
        }
    }

    pub fn set_kinematics(&mut self, pos : Position, vel : Velocity, acc : Acceleration){
        self.pos = pos;
        self.vel = vel;
        self.acc = acc;
    }

    pub fn set_nav_gain(&mut self, nav_gain : f64){
        self.gain = nav_gain;
    }

    pub fn set_time_of_flight(&mut self, tof : f64){
        self.tof = tof;
    }

    pub fn time_of_flight(&self) -> f64{
        self.tof
    }
}

pub struct Vehicle{
    pub pos              : Position,
    pub vel              : Velocity,
    pub acc              : Acceleration,
    pub att              : Attitude,
    pub flight_path      : f64,
    pub flight_path_rate : f64
}

impl Vehicle{
    pub fn new() ->Self{
        Self { 
            pos              : Position::default(), 
            vel              : Velocity::default(), 
            acc              : Acceleration::default(), 
            att              : Attitude::default(), 
            flight_path      : 0.0, 
            flight_path_rate : 0.0 
        }
    }
}
