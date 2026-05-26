use librecrema::framework::{SafeScheduler, PortValue, CoreNode};
use librecrema::nodes::{PidController, PhaseAngleMapper};

#[test]
fn test_native_safe_pid_bumpless() {
    let mut scheduler = SafeScheduler::new();

    let pid_node = CoreNode::Pid(PidController {
        kp: 10.0,
        ki: 1.0,
        kd: 0.1,
        min_output: 0.0,
        max_output: 100.0,
        integral: 0.0,
        prev_error: 0.0,
        filter_coeff: 1.0,
        filtered_derivative: 0.0,
    });

    // Add safe PID node with inputs [Setpoint, PV, ManualPower, EnableAuto] and output [ControlVal]
    assert!(scheduler.add_node(1, pid_node, 4, 1));
    
    // Manual input setups
    scheduler.set_input(1, 0, PortValue::Float(100.0)); // Setpoint
    scheduler.set_input(1, 1, PortValue::Float(98.0));  // Process Variable
    scheduler.set_input(1, 2, PortValue::Float(25.0));  // Manual power bypass input
    scheduler.set_input(1, 3, PortValue::Bool(false));  // Closed-loop disabled

    scheduler.tick(1000);

    // Assert manual configuration tracking behaves seamlessly (no value jumps)
    let output = scheduler.get_output(1, 0).unwrap();
    assert_eq!(output.float_or(0.0), 25.0);

    // Turn on auto-regulation loop
    scheduler.set_input(1, 3, PortValue::Bool(true));
    scheduler.tick(1000);

    let output_auto = scheduler.get_output(1, 0).unwrap();
    assert!(output_auto.float_or(0.0) > 0.0);
}

#[test]
fn test_phase_angle_sinusoidal_solver() {
    let mut scheduler = SafeScheduler::new();
    let pam_node = CoreNode::PhaseAngle(PhaseAngleMapper { mains_freq_hz: 60, max_triac_delay_us: 7500 });

    assert!(scheduler.add_node(2, pam_node, 1, 1));
    
    // Request 50% average RMS power
    scheduler.set_input(2, 0, PortValue::Float(0.5));
    scheduler.tick(100);

    let delay_us = scheduler.get_output(2, 0).unwrap().int_or(0);
    // Half cycle of 60Hz AC mains is ~8333 microseconds. 50% power delays around the peak (4166us)
    assert!(delay_us > 3500 && delay_us < 4800);
}