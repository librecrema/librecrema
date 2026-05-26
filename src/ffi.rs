use crate::framework::{CoreNode, PortValue, SafeScheduler};
use crate::GLOBAL_SCHEDULER;

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum PortDataTypeC {
    Float = 0,
    Bool = 1,
    Int = 2,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union PortValueC {
    pub f_val: f32,
    pub b_val: bool,
    pub i_val: i32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PortC {
    pub value_type: PortDataTypeC,
    pub value: PortValueC,
}

// Convert safe native representation to C FFI structures
pub(crate) fn native_to_c(val: PortValue) -> PortC {
    match val {
        PortValue::Float(f_val) => PortC {
            value_type: PortDataTypeC::Float,
            value: PortValueC { f_val },
        },
        PortValue::Bool(b_val) => PortC {
            value_type: PortDataTypeC::Bool,
            value: PortValueC { b_val },
        },
        PortValue::Int(i_val) => PortC {
            value_type: PortDataTypeC::Int,
            value: PortValueC { i_val },
        },
    }
}

// Convert FFI structures back to safe native representations
pub(crate) fn c_to_native(val: PortC) -> PortValue {
    match val.value_type {
        PortDataTypeC::Float => PortValue::Float(unsafe { val.value.f_val }),
        PortDataTypeC::Bool => PortValue::Bool(unsafe { val.value.b_val }),
        PortDataTypeC::Int => PortValue::Int(unsafe { val.value.i_val }),
    }
}

// Dynamic FFI Node Initialization APIs
#[no_mangle]
pub extern "C" fn librecrema_init() {
    unsafe {
        let scheduler_ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        *scheduler_ptr = Some(SafeScheduler::new());
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_pid_node(
    node_id: u32,
    kp: f32,
    ki: f32,
    kd: f32,
    min_output: f32,
    max_output: f32,
    filter_coeff: f32,
) -> bool {
    let node = CoreNode::Pid(crate::nodes::PidController {
        kp,
        ki,
        kd,
        min_output,
        max_output,
        integral: 0.0,
        prev_error: 0.0,
        filter_coeff,
        filtered_derivative: 0.0,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 4, 1))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_phase_angle_node(
    node_id: u32,
    mains_freq_hz: u32,
    max_triac_delay_us: u32,
) -> bool {
    let node = CoreNode::PhaseAngle(crate::nodes::PhaseAngleMapper {
        mains_freq_hz,
        max_triac_delay_us,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 1, 1))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_thermostat_node(node_id: u32, deadband: f32) -> bool {
    let node = CoreNode::Thermostat(crate::nodes::Thermostat {
        deadband,
        last_state: false,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 2, 1))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_virtual_pressure_node(
    node_id: u32,
    max_p: f32,
    max_f: f32,
) -> bool {
    let node = CoreNode::VirtualPressure(crate::nodes::VirtualPressureSensor {
        max_pressure_bar: max_p,
        max_flow_ml_min: max_f,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 2, 1))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_virtual_flow_node(node_id: u32, empty_headspace: f32) -> bool {
    let node = CoreNode::VirtualFlow(crate::nodes::VirtualFlowSensor {
        empty_headspace_ml: empty_headspace,
        accumulated_water_ml: 0.0,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 2, 2))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_sequencer_node(
    node_id: u32,
    stages: *const crate::nodes::StageParameters,
    total_stages: u32,
) -> bool {
    if stages.is_null()
        || total_stages == 0
        || total_stages as usize > crate::nodes::MAX_PROFILE_STAGES
    {
        return false;
    }

    let mut stages_arr = [crate::nodes::StageParameters {
        stage: crate::nodes::ExtractionStage::Idle,
        duration_ms: 0,
        target_pressure_bar: 0.0,
        target_temp_c: 0.0,
    }; crate::nodes::MAX_PROFILE_STAGES];

    let slice = unsafe { core::slice::from_raw_parts(stages, total_stages as usize) };
    for (i, stage) in slice.iter().enumerate() {
        stages_arr[i] = *stage;
    }

    let node = CoreNode::Sequencer(crate::nodes::Sequencer {
        stages: stages_arr,
        total_stages: total_stages as usize,
        current_stage_idx: 0,
        elapsed_stage_time_ms: 0,
    });

    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 2, 3))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_safety_node(
    node_id: u32,
    max_temp_c: f32,
    max_pressure_bar: f32,
    timeout_limit_ms: u32,
) -> bool {
    let node = CoreNode::Safety(crate::nodes::SafetyWatchdog {
        max_temp_c,
        max_pressure_bar,
        timeout_limit_ms,
        time_above_limits_ms: 0,
        is_system_fault: false,
    });
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr)
            .as_mut()
            .map_or(false, |s| s.add_node(node_id, node, 2, 2))
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_custom_node(
    node_id: u32,
    inputs: u32,
    outputs: u32,
    update_fn: extern "C" fn(
        state: *mut core::ffi::c_void,
        inputs: *const PortC,
        outputs: *mut PortC,
        dt: u32,
    ),
    state: *mut core::ffi::c_void,
) -> bool {
    let node = CoreNode::Custom { update_fn, state };
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr).as_mut().map_or(false, |s| {
            s.add_node(node_id, node, inputs as usize, outputs as usize)
        })
    }
}

#[no_mangle]
pub extern "C" fn librecrema_add_binding(
    src_node: u32,
    src_port: u32,
    dest_node: u32,
    dest_port: u32,
) -> bool {
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr).as_mut().map_or(false, |s| {
            s.add_binding(src_node, src_port as usize, dest_node, dest_port as usize)
        })
    }
}

// Fixed signature: Accepts the PortC struct by reference to prevent FFI value constraints
#[no_mangle]
pub extern "C" fn librecrema_set_port(node_id: u32, port_idx: u32, val: *const PortC) -> bool {
    if val.is_null() {
        return false;
    }
    unsafe {
        let val_deref = *val;
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        (*ptr).as_mut().map_or(false, |s| {
            s.set_input(node_id, port_idx as usize, c_to_native(val_deref))
        })
    }
}

// Fixed signature: Writes structure directly into destination pointer to prevent libffi Union bugs
#[no_mangle]
pub extern "C" fn librecrema_get_port(node_id: u32, port_idx: u32, out_val: *mut PortC) -> bool {
    if out_val.is_null() {
        return false;
    }
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        if let Some(ref scheduler) = *ptr {
            if let Some(val) = scheduler.get_output(node_id, port_idx as usize) {
                *out_val = native_to_c(val);
                return true;
            }
        }
        false
    }
}

#[no_mangle]
pub extern "C" fn librecrema_tick(delta_time_ms: u32) {
    unsafe {
        let ptr = core::ptr::addr_of_mut!(GLOBAL_SCHEDULER);
        if let Some(ref mut scheduler) = *ptr {
            scheduler.tick(delta_time_ms);
        }
    }
}
