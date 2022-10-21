use bevy::prelude::Component;
use game_utils::{
    control_axis::{
        ControlAxis,
        AxisContribution,
    }, 
    dimension3::Dimension3,
    toggle::Toggle,
};
use pid_controller::PID;
use inertial_measurement::IMU;





#[derive(Component)]
pub struct Power{inner: Toggle}
impl Power{
    pub fn new(enabled: bool) -> Self{Self{inner: Toggle::new(enabled)}}
    pub fn enabled(self: &Self) -> bool{self.inner.enabled()}
    pub fn toggle(self: &mut Self){self.inner.toggle()}
}

#[derive(Component)]
pub struct LinearAssist{inner: Toggle}
impl LinearAssist{
    pub fn new(enabled: bool) -> Self{Self{inner: Toggle::new(enabled)}}
    pub fn enabled(self: &Self) -> bool{self.inner.enabled()}
    pub fn toggle(self: &mut Self){self.inner.toggle()}
    pub fn inner(self: &Self) -> &Toggle{&self.inner}
}

#[derive(Component)]
pub struct RotationalAssist{inner: Toggle}
impl RotationalAssist{
    pub fn new(enabled: bool) -> Self{Self{inner: Toggle::new(enabled)}}
    pub fn enabled(self: &Self) -> bool{self.inner.enabled()}
    pub fn toggle(self: &mut Self){self.inner.toggle()}
    pub fn inner(self: &Self) -> &Toggle{&self.inner}
}

#[derive(Component)]
pub struct AutonomousMode{inner: Toggle}
impl AutonomousMode{
    pub fn new(enabled: bool) -> Self{Self{inner: Toggle::new(enabled)}}
    pub fn enabled(self: &Self) -> bool{self.inner.enabled()}
}

#[derive(Component)]
pub struct PID6DOF{inner: ControlAxis<Dimension3<PID<f32>>>}
impl PID6DOF{
    pub fn new(control_axis: ControlAxis<Dimension3<PID<f32>>>) -> Self{Self{inner: control_axis}}
    pub fn inner_mut(self: &mut Self) -> &mut ControlAxis<Dimension3<PID<f32>>>{&mut self.inner}
}

#[derive(Component)]
pub struct MaxVelocity{inner: ControlAxis<Dimension3<f32>>}
impl MaxVelocity{
    pub fn new(control_axis: ControlAxis<Dimension3<f32>>) -> Self{Self{inner: control_axis}}
    pub fn inner(self: &Self) -> &ControlAxis<Dimension3<f32>>{&self.inner}
}

#[derive(Component)]
pub struct PilotAxisInput{inner: ControlAxis<Dimension3<f32>>}
impl PilotAxisInput{
    pub fn new(control_axis: ControlAxis<Dimension3<f32>>) -> Self{Self{inner: control_axis}}
    pub fn inner(self: &Self) -> &ControlAxis<Dimension3<f32>>{&self.inner}
    pub fn inner_mut(self: &mut Self) -> &mut ControlAxis<Dimension3<f32>>{&mut self.inner}
}

#[derive(Component)]
pub struct InertialMeasurementUnit{inner: IMU<f32>}
impl InertialMeasurementUnit{
    pub fn new(imu: IMU<f32>) -> Self{Self{inner: imu}}
    pub fn velocity(self: &Self) -> &ControlAxis<Dimension3<f32>>{self.inner.velocity()}
}

#[derive(Component)]
pub struct GSafety{
    inner: Toggle,
    max_acceleration: ControlAxis<Dimension3<AxisContribution<f32>>>,
}
impl GSafety{
    pub fn new(
        enabled: bool, 
        max_acceleration: ControlAxis<Dimension3<AxisContribution<f32>>>,
    ) -> Self{
        Self{
            inner: Toggle::new(enabled),
            max_acceleration
        }
    }
    pub fn enabled(self: &Self) -> bool{
        self.inner.enabled()
    }
    pub fn max_acceleration(self: &Self) -> &ControlAxis<Dimension3<AxisContribution<f32>>>{
        &self.max_acceleration
    }
}