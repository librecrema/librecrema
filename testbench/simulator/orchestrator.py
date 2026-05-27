import time
from typing import Dict, List, Callable, Tuple
from enum import IntEnum
from simulator.core import EnvironmentState, FluidState
from simulator.profiles import MachineProfile, PROFILES, TempControl, PumpControl, PumpType
from simulator.thermodynamics import ThermalMass
from simulator.hydraulics import HydraulicTopology, Pipe, SolenoidValve, OverPressureValve, VibratoryPump, RotaryVanePump, Drain
from simulator.puck import CoffeePuck
from simulator.actuators import SolidStateRelay, Triac, HeatingElement
from simulator.sensors import TemperatureSensor, PressureTransducer, VolumetricFlowmeter

class ShotState(IntEnum):
    IDLE = 0; PRIMING = 1; PRE_INFUSION = 2; RAMP_TO_PRESSURE = 3; EXTRACTION = 4; DECLINE = 5; COMPLETE = 6

class MockRustCore:
    def __init__(self, profile: MachineProfile, start_empty: bool = False):
        self.profile = profile
        self.target_brew_temp = 93.0
        self.target_steam_temp = 130.0 if profile.has_steam_boiler else 0.0
        self.target_pressure = 0.0
        self.current_brew_heater_pwm = 0.0
        self.current_steam_heater_pwm = 0.0
        self.current_pump_delay_us = 8333
        self.state = ShotState.PRIMING if start_empty else ShotState.IDLE
        self.state_start_time = 0.0

    def tick(self, dt_s: float, current_time_s: float, current_brew_temp: float, current_steam_temp: float, current_pressure: float, current_flow: float):
        if self.state == ShotState.PRIMING:
            self.target_pressure = 9.0
            if current_flow > 0:
                self.state = ShotState.IDLE
                self.target_pressure = 0.0
        elif self.state == ShotState.IDLE:
            self.target_pressure = 0.0
            if current_time_s >= 60.0:
                self.state = ShotState.PRE_INFUSION
                self.state_start_time = current_time_s
        elif self.state == ShotState.PRE_INFUSION:
            self.target_pressure = 2.0
            if current_time_s - self.state_start_time >= 5.0:
                self.state = ShotState.RAMP_TO_PRESSURE
                self.state_start_time = current_time_s
        elif self.state == ShotState.RAMP_TO_PRESSURE:
            elapsed = current_time_s - self.state_start_time
            self.target_pressure = 2.0 + (7.0 * (elapsed / 3.0))
            if elapsed >= 3.0:
                self.state = ShotState.EXTRACTION
                self.state_start_time = current_time_s
        elif self.state == ShotState.EXTRACTION:
            self.target_pressure = 9.0
            if current_time_s - self.state_start_time >= 20.0:
                self.state = ShotState.DECLINE
                self.state_start_time = current_time_s
        elif self.state == ShotState.DECLINE:
            self.target_pressure = 4.0
            if current_time_s - self.state_start_time >= 5.0:
                self.state = ShotState.COMPLETE
                self.state_start_time = current_time_s
        elif self.state == ShotState.COMPLETE:
            self.target_pressure = 0.0

        if self.profile.brew_temp_control == TempControl.PID:
            error_brew = self.target_brew_temp - current_brew_temp
            self.current_brew_heater_pwm = max(0.0, min(1.0, error_brew * 0.15))
        elif self.profile.brew_temp_control == TempControl.THERMOSTAT:
            if current_brew_temp <= self.target_brew_temp - self.profile.brew_thermostat_deadband_c:
                self.current_brew_heater_pwm = 1.0
            elif current_brew_temp >= self.target_brew_temp:
                self.current_brew_heater_pwm = 0.0

        if self.profile.has_steam_boiler:
            if self.profile.steam_temp_control == TempControl.PID:
                error_steam = self.target_steam_temp - current_steam_temp
                self.current_steam_heater_pwm = max(0.0, min(1.0, error_steam * 0.10))
            elif self.profile.steam_temp_control == TempControl.THERMOSTAT:
                if current_steam_temp <= self.target_steam_temp - self.profile.steam_thermostat_deadband_c:
                    self.current_steam_heater_pwm = 1.0
                elif current_steam_temp >= self.target_steam_temp:
                    self.current_steam_heater_pwm = 0.0

        if self.profile.pump_control == PumpControl.PROPORTIONAL:
            if current_pressure < self.target_pressure - 0.2:
                self.current_pump_delay_us = 3000
            elif current_pressure > self.target_pressure:
                self.current_pump_delay_us = 8333
            else:
                self.current_pump_delay_us = 5000
        elif self.profile.pump_control == PumpControl.ON_OFF:
            if self.target_pressure > 0.1:
                self.current_pump_delay_us = 0
            else:
                self.current_pump_delay_us = 8333


class SimulationOrchestrator:
    def __init__(self, profile_name: str = "default", start_empty: bool = False):
        if profile_name not in PROFILES:
            raise ValueError(f"Profile {profile_name} not found.")
        self.profile = PROFILES[profile_name]

        self.sim_tick_s = 0.0001
        self.core_tick_s = 0.01
        self.steps_per_core_tick = int(self.core_tick_s / self.sim_tick_s)

        self.environment = EnvironmentState(
            ambient_temperature_c=22.0, mains_voltage_ac=120.0, mains_frequency_hz=60.0
        )

        initial_temp = self.environment.ambient_temperature_c if start_empty else 93
        initial_fill = 0.0 if start_empty else None

        # --- Build Hydraulic Graph Nodes ---
        if self.profile.pump_type == PumpType.VIBRATORY:
            pump = VibratoryPump("Pump", self.profile.pump_max_pressure_bar, 10.0, self.profile.pump_em_lag_s)
        else:
            pump = RotaryVanePump("Pump", self.profile.pump_max_pressure_bar, 15.0)

        self.pump_pipe = Pipe("PumpPipe", self.profile.system_compliance_ml_per_bar)
        self.opv = OverPressureValve("OPV", self.profile.opv_limit_bar)

        self.brew_boiler = ThermalMass("BrewBoiler", self.profile.brew_metal_thermal_capacity_j_k,
            self.profile.brew_thermal_dissipation_w_k, self.profile.brew_boiler_max_volume_ml, initial_temp, initial_fill)

        self.group_head = ThermalMass("GroupHead", self.profile.group_head_metal_thermal_capacity_j_k,
            self.profile.group_head_thermal_dissipation_w_k, self.profile.group_head_max_volume_ml, 50, initial_fill)

        self.group_valve = SolenoidValve("GroupValve", is_3_way=self.profile.has_3_way_solenoid)
        self.puck = CoffeePuck("Puck", 18.0, 450.0)
        self.puck.is_priming_mode = start_empty
        self.drip_tray = Drain("DripTray")
        self.cup = Drain("Cup")

        # --- Wire the Topology Graph Explicitly ---
        pump.connect("main", self.pump_pipe)
        self.pump_pipe.connect("main", self.opv)

        self.opv.connect("main", self.brew_boiler)
        self.opv.connect("bypass", self.drip_tray) # Excess OPV pressure vents back to tank/drain

        self.brew_boiler.connect("group", self.group_head)
        # self.brew_boiler.connect("steam", steam_wand) # Readily supports adding a steam branch

        self.group_head.connect("main", self.group_valve)
        self.group_valve.connect("main", self.puck)
        if self.group_valve.is_3_way:
            self.group_valve.connect("drain", self.drip_tray)

        self.puck.connect("main", self.cup)

        self.topology = HydraulicTopology(pump)

        # Standalone Steam Boiler (Not hydraulically connected in this specific profile)
        self.steam_boiler = None
        if self.profile.has_steam_boiler:
            self.steam_boiler = ThermalMass("SteamBoiler", self.profile.steam_metal_thermal_capacity_j_k,
                self.profile.steam_thermal_dissipation_w_k, self.profile.steam_boiler_max_volume_ml, initial_temp, initial_fill)

        # Actuators
        self.brew_ssr = SolidStateRelay(pwm_period_s=1.0)
        self.brew_heater = HeatingElement(self.profile.brew_heater_power_w)
        self.steam_ssr = SolidStateRelay(pwm_period_s=1.0)
        self.steam_heater = HeatingElement(self.profile.steam_heater_power_w)
        self.pump_triac = Triac()

        # Sensors (Wired dynamically to nodes/links via lambdas)
        self.temp_sensor_brew = TemperatureSensor()
        self.temp_sensor_brew.attach(lambda: self.brew_boiler.temperature_c)

        self.temp_sensor_steam = TemperatureSensor()
        if self.steam_boiler:
            self.temp_sensor_steam.attach(lambda: self.steam_boiler.temperature_c)
        else:
            self.temp_sensor_steam.attach(lambda: self.environment.ambient_temperature_c)

        self.pressure_sensor = PressureTransducer()
        self.pressure_sensor.attach(lambda: self.topology.links.get("GroupHead_main", FluidState()).pressure_bar)

        self.flowmeter = VolumetricFlowmeter(pulses_per_ml=2.0)
        self.flowmeter.attach(lambda: self.topology.links.get("Puck_main", FluidState()).flow_rate_ml_s)

        self.rust_core = MockRustCore(self.profile, start_empty)

        self.telemetry_sources: Dict[str, Callable[[], float]] = {}
        self.telemetry_data: Dict[str, List[float]] = {"time": []}
        self.telemetry_groups: Dict[str, List[str]] = {}

    def add_telemetry_point(self, name: str, source_callable: Callable[[], float], group: str = None):
        """Registers a telemetry point. If 'group' is provided, points with the same group share a plot."""
        self.telemetry_sources[name] = source_callable
        self.telemetry_data[name] = []

        group_name = group if group else name
        if group_name not in self.telemetry_groups:
            self.telemetry_groups[group_name] = []
        self.telemetry_groups[group_name].append(name)

    def run(self, duration_s: float) -> Tuple[Dict[str, List[float]], Dict[str, List[str]]]:
        current_time = 0.0
        print(f"Starting Graph-Topology simulation for {duration_s} seconds. Profile: {self.profile.name}")

        while current_time < duration_s:
            # --- 1. RUST CONTROL CORE (100Hz) ---
            measured_brew_temp = self.temp_sensor_brew.read()
            measured_steam_temp = self.temp_sensor_steam.read()
            measured_pressure = self.pressure_sensor.read()
            measured_flow_ml_s = (self.flowmeter.read_pulses() / self.flowmeter.pulses_per_ml) / self.core_tick_s

            self.rust_core.tick(self.core_tick_s, current_time, measured_brew_temp, measured_steam_temp, measured_pressure, measured_flow_ml_s)

            self.brew_ssr.apply_logic(self.rust_core.current_brew_heater_pwm)
            self.steam_ssr.apply_logic(self.rust_core.current_steam_heater_pwm)
            self.pump_triac.apply_delay(self.rust_core.current_pump_delay_us)

            # --- Dynamic Telemetry Logging ---
            self.telemetry_data["time"].append(current_time)
            for name, source_fn in self.telemetry_sources.items():
                try:
                    self.telemetry_data[name].append(source_fn())
                except Exception:
                    self.telemetry_data[name].append(0.0)

            # --- 2. HIGH-RES PHYSICS ENGINE (10kHz) ---
            for _ in range(self.steps_per_core_tick):
                self.environment.current_time_s += self.sim_tick_s
                current_time += self.sim_tick_s

                # A. Actuators
                brew_ssr_active = self.brew_ssr.get_output_state(self.environment)
                brew_applied_power_w = self.brew_heater.get_applied_power_w(brew_ssr_active, self.environment)

                pump_rms_multiplier = self.pump_triac.get_rms_voltage_multiplier(self.environment)
                target_pump_power = pump_rms_multiplier ** 2

                is_shot_active = self.rust_core.target_pressure > 0.0
                self.group_valve.apply_logic(is_shot_active)
                self.puck.is_priming_mode = (self.rust_core.state == ShotState.PRIMING)

                # B. Thermodynamics Prep
                self.brew_boiler.apply_conduction(self.group_head, self.profile.boiler_to_group_conductivity_w_k, self.sim_tick_s)
                self.brew_boiler.set_thermal_inputs(brew_applied_power_w, self.environment)
                self.group_head.set_thermal_inputs(self.profile.group_head_heater_power_w, self.environment)

                # C. Topological Graph Fluid Sweep
                self.topology.step(self.sim_tick_s, target_pump_power)

                # D. Steam Boiler
                if self.steam_boiler:
                    steam_ssr_active = self.steam_ssr.get_output_state(self.environment)
                    steam_applied_power_w = self.steam_heater.get_applied_power_w(steam_ssr_active, self.environment)
                    self.steam_boiler.set_thermal_inputs(steam_applied_power_w, self.environment)
                    self.steam_boiler.process_fluid(self.sim_tick_s, FluidState(0,0,self.environment.ambient_temperature_c))

                # E. Update Sensors (Now they poll their own attached sources!)
                self.temp_sensor_brew.step(self.sim_tick_s)
                self.temp_sensor_steam.step(self.sim_tick_s)
                self.pressure_sensor.step(self.sim_tick_s)
                self.flowmeter.step(self.sim_tick_s)

        print("Simulation complete.")
        return self.telemetry_data, self.telemetry_groups