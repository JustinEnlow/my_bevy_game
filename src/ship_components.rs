use bevy::prelude::Component;
use game_utils::{
    toggle::Toggle, 
    control_axis::{
        ControlAxis, 
        AxisContribution
    }, 
    dimension3::Dimension3
};
use pid_controller::PID;
use inertial_measurement::IMU;





//#[derive(Component)]
//pub struct Power{enabled: bool}
//impl Toggle for Power{
//    fn new(enabled: bool) -> Self{Self{enabled}}
//    fn enabled(self: &Self) -> bool{self.enabled}
//    fn toggle(self: &mut Self){self.enabled = !self.enabled}
//}

#[derive(Component)]
//pub struct Power{pub toggle: Toggle}
pub struct Power{enabled: bool}
impl Power{
    pub fn new(enabled: bool) -> Self{Self{enabled}}
}
impl Toggle for Power{
    fn enabled(self: &Self) -> bool{self.enabled}
    fn toggle(self: &mut Self){self.enabled = !self.enabled}    
}

#[derive(Component)]
//pub struct LinearAssist{pub toggle: Toggle}
pub struct LinearAssist{enabled: bool}
impl LinearAssist{
    pub fn new(enabled: bool) -> Self{Self{enabled}}
}
impl Toggle for LinearAssist{
    fn enabled(self: &Self) -> bool{self.enabled}
    fn toggle(self: &mut Self){self.enabled = !self.enabled}
}

#[derive(Component)]
//pub struct RotationalAssist{pub toggle: Toggle}
pub struct RotationalAssist{enabled: bool}
impl RotationalAssist{
    pub fn new(enabled: bool) -> Self{Self{enabled}}
}
impl Toggle for RotationalAssist{
    fn enabled(self: &Self) -> bool{self.enabled}
    fn toggle(self: &mut Self){self.enabled = !self.enabled}
}

#[derive(Component)]
//pub struct AutonomousMode{pub toggle: Toggle}
pub struct AutonomousMode{enabled: bool}
impl AutonomousMode{
    pub fn new(enabled: bool) -> Self{Self{enabled}}
}
impl Toggle for AutonomousMode{
    fn enabled(self: &Self) -> bool{self.enabled}
    fn toggle(self: &mut Self){self.enabled = !self.enabled}
}

#[derive(Component)]
pub struct PID6DOF{pub control_axis: ControlAxis<Dimension3<PID<f32>>>}

#[derive(Component)]
pub struct MaxVelocity(pub ControlAxis<Dimension3<f32>>);

#[derive(Component)]
pub struct PilotAxisInput(pub ControlAxis<Dimension3<f32>>);

#[derive(Component)]
pub struct InertialMeasurementUnit(pub IMU<f32>);

#[derive(Component)]
pub struct GSafety{
    //pub toggle: Toggle,
    enabled: bool,
    pub max_acceleration: ControlAxis<Dimension3<AxisContribution<f32>>>,
}
impl GSafety{
    pub fn new(
        enabled: bool, 
        max_acceleration: ControlAxis<Dimension3<AxisContribution<f32>>>
    ) -> Self{
        Self{
            enabled, 
            max_acceleration
        }
    }
}
impl Toggle for GSafety{
    fn enabled(self: &Self) -> bool{self.enabled}
    fn toggle(self: &mut Self){self.enabled = !self.enabled}
}