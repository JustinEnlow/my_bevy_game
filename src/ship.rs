use bevy::{input::{keyboard::KeyCode, Input}, prelude::*};
use crate::ship_components::*;
use game_utils::{
    control_axis::{ControlAxis, AxisContribution}, 
    dimension3::Dimension3,
    //toggle::{Toggle/*, TToggle*/},
};
use pid_controller::PID;
use inertial_measurement::IMU;



pub struct ShipPlugin;

impl Plugin for ShipPlugin{
    fn build(&self, app: &mut App){
        app.add_startup_system(spawn_ship_entity)
            .add_system(input_handling_system)
            .add_system(flight_control_system)
        ;
    }
}



pub fn spawn_ship_entity(mut commands: Commands){
    commands.spawn()
        .insert(Power::new(false))
        .insert(LinearAssist::new(true))
        .insert(RotationalAssist::new(true))
        .insert(AutonomousMode::new(false))
        .insert(
            PID6DOF::new(
                ControlAxis::new(
                    Dimension3::new(
                        PID::new(100.0, 0.0, 0.0, None), 
                        PID::new(100.0, 0.0, 0.0, None), 
                        PID::new(100.0, 0.0, 0.0, None)
                    ), 
                    Dimension3::new(
                        PID::new(100.0, 0.0, 0.0, None), 
                        PID::new(100.0, 0.0, 0.0, None), 
                        PID::new(100.0, 0.0, 0.0, None)
                    )
                )
            )
        )
        .insert(
            MaxVelocity::new(
                ControlAxis::new(
                    Dimension3::new(100.0, 100.0, 100.0),
                    Dimension3::new(100.0, 100.0, 100.0)
                )
            )
        )
        .insert(
            PilotAxisInput::new(
                ControlAxis::new(
                    Dimension3::new(1.0, 0.0, 0.0),
                    Dimension3::new(0.0, 0.0, 0.0)
                )
            )
        )
        .insert(
            InertialMeasurementUnit::new(
                IMU::new(
                    ControlAxis::new(
                        Dimension3::new(0.0, 0.0, 0.0),
                        Dimension3::new(0.0, 0.0, 0.0)
                    ),
                    ControlAxis::new(
                        Dimension3::new(0.0, 0.0, 0.0),
                        Dimension3::new(0.0, 0.0, 0.0)
                    ),
                    ControlAxis::new(
                        Dimension3::default(0.0),
                        Dimension3::default(0.0)
                    ),
                )
            )
        )
        .insert(
            GSafety::new(
                false,
                ControlAxis::new(
                    Dimension3::new(
                        AxisContribution::new(50.0, 50.0),
                        AxisContribution::new(50.0, 50.0),
                        AxisContribution::new(50.0, 50.0),
                    ),
                    Dimension3::new(
                        AxisContribution::new(0.0, 0.0),
                        AxisContribution::new(0.0, 0.0),
                        AxisContribution::new(0.0, 0.0),
                    )
                )
            )
        )
    ;
}



pub fn flight_control_system(
    time: Res<Time>,
    mut query: Query<(
        &Power, 
        &LinearAssist, 
        &RotationalAssist, 
        &AutonomousMode, 
        &mut PID6DOF, 
        &MaxVelocity,
        &InertialMeasurementUnit, 
        &PilotAxisInput, 
        &GSafety, 
    )>
){
    for (
        power,
        linear_assist,
        rotational_assist,
        autonomous_mode,
        mut pid6dof,
        max_velocity,
        imu,
        input,
        gsafety,
    ) in query.iter_mut(){

        if power.enabled() == false{return;}

        // placeholder value. actual values will be determined  by thruster suite/propulsion control system
        let available_acceleration = ControlAxis::new(
            Dimension3::new(
                AxisContribution::new(0.0, 0.0),
                AxisContribution::new(0.0, 0.0),
                AxisContribution::new(0.0, 0.0),
            ),
            Dimension3::new(
                AxisContribution::new(0.0, 0.0),
                AxisContribution::new(0.0, 0.0),
                AxisContribution::new(0.0, 0.0),
            )
        );

        let mut desired_acceleration = game_utils::sum_d3_control_axes(
            if autonomous_mode.enabled(){
                flight_controller::feedforward_controller::calculate_autonomous_mode_acceleration(
                    &ControlAxis::default(Dimension3::default(0.0)),
                    &ControlAxis::default(Dimension3::default(0.0)),
                    imu.velocity(),
                    max_velocity.inner(),
                    &available_acceleration,
                    time.delta_seconds(),
                )
            }else{
                flight_controller::feedforward_controller::calculate_pilot_control_mode_acceleration(
                    input.inner(),
                    linear_assist.inner(),
                    rotational_assist.inner(),
                    max_velocity.inner(),
                    imu.velocity(),
                    &available_acceleration,
                    time.delta_seconds(),
                )
            },
            flight_controller::feedback_controller::calculate(
                &ControlAxis::default(Dimension3::default(0.0)), 
                &ControlAxis::default(Dimension3::default(0.0)),    // current translation/rotation
                pid6dof.inner_mut(), 
                &available_acceleration,
                time.delta_seconds(),
            )
        );
        
        if gsafety.enabled(){
            desired_acceleration = flight_controller::g_force_safety::process(
                &desired_acceleration,
                gsafety.max_acceleration()
            )
        }

        // propulsion control
        // feed resultant forces to physics system
        println!("{}", desired_acceleration.linear().x());
    }
}




pub fn input_handling_system(
    keyboard_input: Res<Input<KeyCode>>, 
    mut query: Query<(
        &mut LinearAssist, 
        &mut RotationalAssist, 
        &mut PilotAxisInput,
        &mut Power
    )>
){
    for (
        mut linear_assist, 
        mut rotational_assist,
        mut pilot_axis_input,
        mut power
    ) in query.iter_mut(){
        
        //power
        if keyboard_input.pressed(KeyCode::P){
            power.toggle();
            println!("power: {}", power.enabled());
        }

        //linear assist
        if keyboard_input.pressed(KeyCode::Z){
            linear_assist.toggle();
            println!("linear assist: {}", linear_assist.enabled());
        }
        //rotational assist
        if keyboard_input.pressed(KeyCode::X){
            rotational_assist.toggle();
            println!("rotational assist: {}", rotational_assist.enabled());
        }

        //forward
        if keyboard_input.just_pressed(KeyCode::E){
            pilot_axis_input.inner_mut().linear_mut().set_z(1.0)
        }
        if keyboard_input.just_released(KeyCode::E){
            pilot_axis_input.inner_mut().linear_mut().set_z(0.0)
        }
        //backward
        if keyboard_input.just_pressed(KeyCode::D){
            pilot_axis_input.inner_mut().linear_mut().set_z(-1.0)
        }
        if keyboard_input.just_released(KeyCode::D){
            pilot_axis_input.inner_mut().linear_mut().set_z(0.0)
        }

        //right
        if keyboard_input.just_pressed(KeyCode::F){
            pilot_axis_input.inner_mut().linear_mut().set_x(1.0)
        }
        if keyboard_input.just_released(KeyCode::F){
            pilot_axis_input.inner_mut().linear_mut().set_x(0.0)
        }
        //left
        if keyboard_input.just_pressed(KeyCode::S){
            pilot_axis_input.inner_mut().linear_mut().set_x(-1.0)
        }
        if keyboard_input.just_released(KeyCode::S){
            pilot_axis_input.inner_mut().linear_mut().set_x(0.0)
        }

        //up
        if keyboard_input.just_pressed(KeyCode::Q){
            pilot_axis_input.inner_mut().linear_mut().set_y(1.0)
        }
        if keyboard_input.just_released(KeyCode::Q){
            pilot_axis_input.inner_mut().linear_mut().set_y(0.0)
        }
        //down
        if keyboard_input.just_pressed(KeyCode::A){
            pilot_axis_input.inner_mut().linear_mut().set_y(-1.0)
        }
        if keyboard_input.just_released(KeyCode::A){
            pilot_axis_input.inner_mut().linear_mut().set_y(0.0)
        }


        //pitch up

        //pitch down

        //yaw right

        //yaw left

        //roll right
        
        //roll left
    }
}






// chain output of one system to input of another.
//pub fn test_system_chain_1() -> String{
//    "hey".to_string()
//}
// 
//pub fn test_system_chain_2(In(mut string): In<String>){
//    string.push_str(", bitch!");
//    println!("{string}")
//}