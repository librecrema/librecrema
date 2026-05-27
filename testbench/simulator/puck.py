from simulator.core import FluidState, HydraulicNode
from typing import Dict

class CoffeePuck(HydraulicNode):
    def __init__(self, name: str, dose_g: float, grind_size_microns: float):
        super().__init__(name)
        self.dose_g = dose_g
        self.grind_size_microns = grind_size_microns

        self.saturation_ml = 0.0
        self.max_saturation_ml = dose_g * 1.2
        self.total_yield_ml = 0.0
        self.is_priming_mode = False

    def update_downstream_resistance(self) -> float:
        if self.is_priming_mode:
            return 0.1  # Blank shot

        # Grind Size Physics: Resistance is inversely proportional to the square of particle size
        # We use 400 microns as our 1.0x baseline reference.
        grind_factor = (400.0 / max(1.0, self.grind_size_microns)) ** 2

        overall_multiplier = grind_factor

        if self.saturation_ml < self.max_saturation_ml:
            swell_factor = self.saturation_ml / max(0.1, self.max_saturation_ml)
            base_res = 1.0 * overall_multiplier
            peak_res = 6.0 * overall_multiplier
            return base_res + (swell_factor * (peak_res - base_res))

        peak_res = 6.0 * overall_multiplier
        end_res = 3.5 * overall_multiplier
        erosion_factor = min(1.0, self.total_yield_ml / 40.0)
        return peak_res - (erosion_factor * (peak_res - end_res))

    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        if self.is_priming_mode:
            return self.distribute_flow(fluid_in.flow_rate_ml_s, 0.0, fluid_in.temperature_c)

        water_entering_ml = fluid_in.flow_rate_ml_s * dt

        if self.saturation_ml < self.max_saturation_ml:
            self.saturation_ml += water_entering_ml
            cup_flow = 0.0
        else:
            cup_flow = fluid_in.flow_rate_ml_s
            self.total_yield_ml += cup_flow * dt

        return self.distribute_flow(cup_flow, 0.0, fluid_in.temperature_c)