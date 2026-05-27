from dataclasses import dataclass
from typing import Optional
from enum import Enum

class PumpType(Enum):
    VIBRATORY = "vibratory"
    ROTARY = "rotary"
    GEAR = "gear"

class TempControl(Enum):
    PID = "pid"
    THERMOSTAT = "thermostat"

class PumpControl(Enum):
    PROPORTIONAL = "proportional"
    ON_OFF = "on_off"

@dataclass
class MachineProfile:
    name: str

    # Control Capabilities
    brew_temp_control: TempControl
    steam_temp_control: Optional[TempControl]
    brew_thermostat_deadband_c: float
    steam_thermostat_deadband_c: float
    pump_control: PumpControl

    # Heating configuration
    has_steam_boiler: bool
    brew_heater_power_w: float
    steam_heater_power_w: float

    # Thermal dynamics (METAL Thermal Capacity in Joules/Kelvin)
    # The simulator will dynamically add water capacity (4.184 J/gK) based on fill level
    brew_metal_thermal_capacity_j_k: float
    steam_metal_thermal_capacity_j_k: float

    # Boiler Internal Volumes
    brew_boiler_max_volume_ml: float
    steam_boiler_max_volume_ml: float

    # Dissipation is Watts lost per degree Kelvin above ambient (W/K)
    brew_thermal_dissipation_w_k: float
    steam_thermal_dissipation_w_k: float

    # Group Head configuration
    group_head_metal_thermal_capacity_j_k: float
    group_head_thermal_dissipation_w_k: float
    group_head_heater_power_w: float
    group_head_max_volume_ml: float          # Internal piping volume inside group
    boiler_to_group_conductivity_w_k: float  # How tightly coupled the boiler is to the group head (W/K)

    # Hydraulic configuration
    pump_type: PumpType
    pump_max_pressure_bar: float
    pump_em_lag_s: float
    system_compliance_ml_per_bar: float
    opv_limit_bar: Optional[float]

    has_3_way_solenoid: bool


# Predefined Industry Profiles
PROFILES = {
    "default": MachineProfile(
        name="Simulation Coffee Machine",
        brew_temp_control=TempControl.PID,
        steam_temp_control=None,
        brew_thermostat_deadband_c=0.0,
        steam_thermostat_deadband_c=0.0,
        pump_control=PumpControl.PROPORTIONAL,

        has_steam_boiler=False,
        brew_heater_power_w=1600.0,
        steam_heater_power_w=0.0,

        brew_metal_thermal_capacity_j_k=600.0,
        steam_metal_thermal_capacity_j_k=0.0,
        brew_boiler_max_volume_ml=15.0,
        steam_boiler_max_volume_ml=0.0,

        brew_thermal_dissipation_w_k=0.80,
        steam_thermal_dissipation_w_k=0.0,

        group_head_metal_thermal_capacity_j_k=500.0,
        group_head_thermal_dissipation_w_k=1.5,
        group_head_heater_power_w=0.0,
        group_head_max_volume_ml=0.0,
        boiler_to_group_conductivity_w_k=8.0, # High conductivity, strongly coupled

        pump_type=PumpType.VIBRATORY,
        pump_max_pressure_bar=15.0,
        pump_em_lag_s=0.15,
        system_compliance_ml_per_bar=0.6,
        opv_limit_bar=15.0,
        has_3_way_solenoid=False
    ),
    "gaggia_anima": MachineProfile(
        name="Gaggia Anima",
        
        # Control Capabilities
        brew_temp_control=TempControl.PID,
        steam_temp_control=None,
        brew_thermostat_deadband_c=0.0,
        steam_thermostat_deadband_c=0.0,
        pump_control=PumpControl.ON_OFF,

        # Heating configuration
        has_steam_boiler=False,
        brew_heater_power_w=1300.0,
        steam_heater_power_w=0.0,

        # Thermal dynamics
        brew_metal_thermal_capacity_j_k=617.0, 
        steam_metal_thermal_capacity_j_k=0.0,
        
        # Boiler Internal Volumes
        brew_boiler_max_volume_ml=15.0,
        steam_boiler_max_volume_ml=0.0,

        # Dissipation
        brew_thermal_dissipation_w_k=0.60,
        steam_thermal_dissipation_w_k=0.0,

        # Group Head configuration
        group_head_metal_thermal_capacity_j_k=150.0,
        group_head_thermal_dissipation_w_k=0.4,
        group_head_heater_power_w=0.0,
        group_head_max_volume_ml=5.0,
        boiler_to_group_conductivity_w_k=0, # no thermal bridge

        # Hydraulic configuration
        pump_type=PumpType.VIBRATORY,
        pump_max_pressure_bar=15.0,
        pump_em_lag_s=0.15,
        system_compliance_ml_per_bar=0.6,
        opv_limit_bar=16.0,
        
        has_3_way_solenoid=False
    )
}