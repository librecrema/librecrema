from simulator.core import FluidState, EnvironmentState, HydraulicNode
from typing import Dict

C_WATER_J_G_K = 4.184

class ThermalMass(HydraulicNode):
    def __init__(self, name: str, metal_thermal_capacity_j_k: float, dissipation_w_k: float, 
                 max_volume_ml: float, initial_temp_c: float = 22.0, initial_fill_ml: float = None):
        super().__init__(name)
        self.metal_capacity = max(1.0, metal_thermal_capacity_j_k)
        self.dissipation = dissipation_w_k
        self.max_volume_ml = max(0.0, max_volume_ml)
        
        if initial_fill_ml is None:
            self.current_volume_ml = self.max_volume_ml
        else:
            self.current_volume_ml = min(max(0.0, initial_fill_ml), self.max_volume_ml)
            
        self.temperature_c = initial_temp_c
        self.current_power_w = 0.0
        self.current_env = None

    def get_internal_resistance(self) -> float:
        return 0.1 # Boilers/Blocks introduce minor hydraulic resistance

    def set_thermal_inputs(self, power_w: float, env: EnvironmentState):
        self.current_power_w = power_w
        self.current_env = env

    def get_total_thermal_capacity(self) -> float:
        water_capacity = self.current_volume_ml * C_WATER_J_G_K
        return self.metal_capacity + water_capacity

    def apply_conduction(self, other: 'ThermalMass', conductivity_w_k: float, dt: float):
        temp_diff = self.temperature_c - other.temperature_c
        heat_transfer_w = temp_diff * conductivity_w_k
        energy_transfer_j = heat_transfer_w * dt
        
        self.temperature_c -= energy_transfer_j / self.get_total_thermal_capacity()
        other.temperature_c += energy_transfer_j / other.get_total_thermal_capacity()

    def _apply_environmental_heat(self, dt: float, applied_power_w: float, env: EnvironmentState):
        temp_diff = self.temperature_c - env.ambient_temperature_c
        heat_loss_w = temp_diff * self.dissipation
        
        net_energy_j = (applied_power_w - heat_loss_w) * dt
        self.temperature_c += net_energy_j / self.get_total_thermal_capacity()

    def _apply_advection_and_flow(self, dt: float, fluid_in: FluidState) -> float:
        if fluid_in.flow_rate_ml_s <= 0.0:
            return 0.0

        in_ml = fluid_in.flow_rate_ml_s * dt
        
        current_total_j_k = self.get_total_thermal_capacity()
        in_water_j_k = in_ml * C_WATER_J_G_K

        mixed_temp = ((current_total_j_k * self.temperature_c) + 
                      (in_water_j_k * fluid_in.temperature_c)) / (current_total_j_k + in_water_j_k)
        self.temperature_c = mixed_temp
        
        available_space = self.max_volume_ml - self.current_volume_ml
        
        if in_ml <= available_space:
            self.current_volume_ml += in_ml
            out_ml = 0.0
        else:
            out_ml = in_ml - available_space
            self.current_volume_ml = self.max_volume_ml
        return out_ml / dt

    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        if self.current_env:
            self._apply_environmental_heat(dt, self.current_power_w, self.current_env)
            
        out_flow_rate = self._apply_advection_and_flow(dt, fluid_in)
        p_out = max(0.0, fluid_in.pressure_bar - (fluid_in.flow_rate_ml_s * self.get_internal_resistance()))
        
        # Distributes the heated fluid among outputs (e.g., grouphead, steam wand)
        return self.distribute_flow(out_flow_rate, p_out, self.temperature_c)