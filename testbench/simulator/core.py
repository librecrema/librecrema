from dataclasses import dataclass
from typing import Dict, Optional

@dataclass
class EnvironmentState:
    """Holds global environmental variables updated continuously."""
    ambient_temperature_c: float
    mains_voltage_ac: float
    mains_frequency_hz: float
    current_time_s: float = 0.0

@dataclass
class FluidState:
    """The standard packet passed between hydraulic and thermal nodes."""
    pressure_bar: float = 0.0
    flow_rate_ml_s: float = 0.0
    temperature_c: float = 20.0
    
    def clone(self) -> 'FluidState':
        return FluidState(
            pressure_bar=self.pressure_bar, 
            flow_rate_ml_s=self.flow_rate_ml_s, 
            temperature_c=self.temperature_c, 
        )

class HydraulicNode:
    """Graph node representing a component in the machine topology."""
    def __init__(self, name: str):
        self.name = name
        self.outputs: Dict[str, 'HydraulicNode'] = {}
        # Cache of downstream resistances calculated during backward sweep
        self.downstream_resistances: Dict[str, float] = {}

    def connect(self, port_name: str, target: 'HydraulicNode'):
        """Connects an output port to a downstream node."""
        self.outputs[port_name] = target

    def get_internal_resistance(self) -> float:
        """Override to provide inherent resistance of the component."""
        return 0.0

    def calculate_total_downstream_resistance(self) -> float:
        """
        Calculates parallel resistance of all connected output branches.
        1 / R_total = (1 / R_out1) + (1 / R_out2) ...
        """
        if not self.outputs:
            return 1e6 # Dead end (infinite resistance) if not explicitly overridden

        inv_r = 0.0
        for port, node in self.outputs.items():
            r = node.update_downstream_resistance()
            self.downstream_resistances[port] = r
            if r < 1e6: # Ignore completely closed paths
                inv_r += 1.0 / max(1e-6, r)
        
        if inv_r == 0.0:
            return 1e6 # All paths are mechanically blocked
        return 1.0 / inv_r

    def update_downstream_resistance(self) -> float:
        """Backward Sweep step: Returns R_internal + R_downstream_parallel"""
        r_down = self.calculate_total_downstream_resistance()
        return self.get_internal_resistance() + r_down

    def distribute_flow(self, total_flow: float, pressure_bar: float, temperature_c: float) -> Dict[str, FluidState]:
        """Distributes incoming flow to output ports inversely proportional to their resistance."""
        out_fluids = {}
        r_total_down = self.calculate_total_downstream_resistance()

        for port in self.outputs.keys():
            r_port = self.downstream_resistances[port]
            if r_port >= 1e6:
                flow = 0.0
            else:
                # Q_port = Q_total * (R_total / R_port)
                flow = total_flow * (r_total_down / max(1e-6, r_port)) if total_flow > 0 else 0.0
            
            out_fluids[port] = FluidState(
                pressure_bar=pressure_bar, 
                flow_rate_ml_s=flow,
                temperature_c=temperature_c,
            )
        return out_fluids

    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        """
        Forward Sweep step. Applies internal pressure drops and distributes fluid.
        """
        r_int = self.get_internal_resistance()
        p_out = max(0.0, fluid_in.pressure_bar - (fluid_in.flow_rate_ml_s * r_int))
        return self.distribute_flow(fluid_in.flow_rate_ml_s, p_out, fluid_in.temperature_c)
