use crate::framework::{Node, PortValue};

// ==========================================
// 1. PID CONTROLLER MODULE
// ==========================================
pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub min_output: f32,
    pub max_output: f32,
    pub integral: f32,
    pub prev_error: f32,
    pub filter_coeff: f32,
    pub filtered_derivative: f32,
}

impl Node for PidController {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32) {
        let setpoint = inputs[0].float_or(0.0);
        let pv = inputs[1].float_or(0.0);
        let manual_power = inputs[2].float_or(0.0);
        let auto_enabled = inputs[3].bool_or(false);

        let dt = (delta_time_ms as f32) / 1000.0;
        if dt <= 0.0001 {
            return;
        }

        let error = setpoint - pv;

        if !auto_enabled {
            // Safe manual state tracking (bumpless transition pre-conditioning)
            self.integral = (manual_power - (self.kp * error)) / (self.ki + 1e-5);
            self.integral = self.integral.clamp(self.min_output, self.max_output);
            self.prev_error = error;
            outputs[0] = PortValue::Float(manual_power.clamp(self.min_output, self.max_output));
            return;
        }

        let p_term = self.kp * error;
        let raw_deriv = (error - self.prev_error) / dt;
        self.filtered_derivative += self.filter_coeff * (raw_deriv - self.filtered_derivative);
        let d_term = self.kd * self.filtered_derivative;

        // Predictive anti-windup clamping evaluation
        let temp_out = p_term + (self.ki * (self.integral + error * dt)) + d_term;
        let saturated = temp_out < self.min_output || temp_out > self.max_output;
        let matching_signs = (error > 0.0 && temp_out > self.max_output)
            || (error < 0.0 && temp_out < self.min_output);

        if !saturated || !matching_signs {
            self.integral += error * dt;
        }

        let final_out =
            (p_term + (self.ki * self.integral) + d_term).clamp(self.min_output, self.max_output);
        self.prev_error = error;
        outputs[0] = PortValue::Float(final_out);
    }
}

// ==========================================
// 2. PHASE ANGLE MAPPER MODULE
// ==========================================
pub struct PhaseAngleMapper {
    pub mains_freq_hz: u32,
    pub max_triac_delay_us: u32,
}

impl Node for PhaseAngleMapper {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], _delta_time_ms: u32) {
        let power = inputs[0].float_or(0.0).clamp(0.0, 1.0);
        let pi = core::f32::consts::PI;

        // Binary Search Solver for transcendental AC root integral
        let mut low = 0.0;
        let mut high = pi;
        let mut alpha = pi / 2.0;

        for _ in 0..10 {
            let p_calculated = 1.0 - (alpha / pi) + libm::sinf(2.0 * alpha) / (2.0 * pi);
            if p_calculated > power {
                low = alpha;
            } else {
                high = alpha;
            }
            alpha = (low + high) / 2.0;
        }

        let half_cycle_us = 1_000_000 / (2 * self.mains_freq_hz);
        let mut delay_us = ((alpha / pi) * half_cycle_us as f32) as u32;

        if delay_us > self.max_triac_delay_us {
            delay_us = self.max_triac_delay_us;
        }
        if power >= 0.99 {
            delay_us = 0;
        }

        outputs[0] = PortValue::Int(delay_us as i32);
    }
}

// ==========================================
// 3. SEQUENCER MODULE
// ==========================================
pub const MAX_PROFILE_STAGES: usize = 8;

#[repr(u32)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ExtractionStage {
    Idle = 0,
    PreInfusion = 1,
    RampToPressure = 2,
    Extraction = 3,
    Decline = 4,
    Complete = 5,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct StageParameters {
    pub stage: ExtractionStage,
    pub duration_ms: u32,
    pub target_pressure_bar: f32,
    pub target_temp_c: f32,
}

pub struct Sequencer {
    pub stages: [StageParameters; MAX_PROFILE_STAGES],
    pub total_stages: usize,
    pub current_stage_idx: usize,
    pub elapsed_stage_time_ms: u32,
}

impl Node for Sequencer {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32) {
        let start_brew = inputs[0].bool_or(false);

        if !start_brew {
            self.current_stage_idx = 0;
            self.elapsed_stage_time_ms = 0;
            outputs[0] = PortValue::Float(0.0); // Target Pressure (0 Bar when idle)
            outputs[1] = PortValue::Float(93.5); // Standby Target Temperature (Allows pre-heating)
            outputs[2] = PortValue::Int(0); // All routing solenoids closed
            return;
        }

        if self.current_stage_idx >= self.total_stages {
            outputs[0] = PortValue::Float(0.0);
            outputs[1] = PortValue::Float(93.5); // Maintain standby temperature
            outputs[2] = PortValue::Int(0b010); // Release pressure
            return;
        }

        let current_stage = &self.stages[self.current_stage_idx];
        self.elapsed_stage_time_ms += delta_time_ms;

        if self.elapsed_stage_time_ms >= current_stage.duration_ms {
            self.current_stage_idx += 1;
            self.elapsed_stage_time_ms = 0;
        }

        if self.current_stage_idx < self.total_stages {
            let stage = &self.stages[self.current_stage_idx];
            outputs[0] = PortValue::Float(stage.target_pressure_bar);
            outputs[1] = PortValue::Float(stage.target_temp_c);
            outputs[2] = PortValue::Int(match stage.stage {
                ExtractionStage::PreInfusion => 0b001,
                ExtractionStage::RampToPressure
                | ExtractionStage::Extraction
                | ExtractionStage::Decline => 0b101,
                _ => 0b000,
            });
        }
    }
}

// ==========================================
// 4. SAFETY WATCHDOG MODULE
// ==========================================
pub struct SafetyWatchdog {
    pub max_temp_c: f32,
    pub max_pressure_bar: f32,
    pub timeout_limit_ms: u32,
    pub time_above_limits_ms: u32,
    pub is_system_fault: bool,
}

impl Node for SafetyWatchdog {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32) {
        let temp = inputs[0].float_or(0.0);
        let pressure = inputs[1].float_or(0.0);

        let over_limit = temp > self.max_temp_c || temp < 0.0 || pressure > self.max_pressure_bar;

        if over_limit {
            self.time_above_limits_ms += delta_time_ms;
            if self.time_above_limits_ms > self.timeout_limit_ms {
                self.is_system_fault = true;
            }
        } else {
            self.time_above_limits_ms = 0;
        }

        outputs[0] = PortValue::Bool(self.is_system_fault); // Heater Cutoff
        outputs[1] = PortValue::Bool(self.is_system_fault); // Pump Cutoff
    }
}

// ==========================================
// 5. THERMOSTAT MODULE
// ==========================================
pub struct Thermostat {
    pub deadband: f32,
    pub last_state: bool,
}

impl Node for Thermostat {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], _delta_time_ms: u32) {
        let setpoint = inputs[0].float_or(0.0);
        let pv = inputs[1].float_or(0.0);

        let lower = setpoint - (self.deadband / 2.0);
        let upper = setpoint + (self.deadband / 2.0);

        if pv < lower {
            self.last_state = true;
        } else if pv > upper {
            self.last_state = false;
        }

        outputs[0] = PortValue::Bool(self.last_state);
    }
}

// ==========================================
// 6. VIRTUAL PRESSURE SENSOR MODULE
// ==========================================
pub struct VirtualPressureSensor {
    pub max_pressure_bar: f32,
    pub max_flow_ml_min: f32,
}

impl Node for VirtualPressureSensor {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], _delta_time_ms: u32) {
        let power = inputs[0].float_or(0.0).clamp(0.0, 1.0);
        let flow = inputs[1].float_or(0.0).max(0.0);

        let flow_ratio = (flow / self.max_flow_ml_min).clamp(0.0, 1.0);
        let p_est = self.max_pressure_bar * (1.0 - flow_ratio * flow_ratio) * power;

        outputs[0] = PortValue::Float(p_est);
    }
}

// ==========================================
// 7. VIRTUAL FLOW SENSOR MODULE
// ==========================================
pub struct VirtualFlowSensor {
    pub empty_headspace_ml: f32,
    pub accumulated_water_ml: f32,
}

impl Node for VirtualFlowSensor {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32) {
        let power = inputs[0].float_or(0.0).clamp(0.0, 1.0);
        let resistance = inputs[1].float_or(1.0).max(0.1);
        let dt_min = (delta_time_ms as f32) / 60000.0;

        let cur_resistance = if self.accumulated_water_ml < self.empty_headspace_ml {
            1.0
        } else {
            resistance
        };

        let flow_rate = (650.0 * power) / cur_resistance;
        self.accumulated_water_ml += flow_rate * dt_min;

        outputs[0] = PortValue::Float(flow_rate);
        outputs[1] = PortValue::Float(self.accumulated_water_ml);
    }
}
