from simulator.core import FluidState, HydraulicNode
from typing import Dict, List, Set

class Pump(HydraulicNode):
    """Base class for pressure/flow generation."""
    def generate_fluid(self, dt: float, power_multiplier: float) -> Dict[str, FluidState]:
        pass

class VibratoryPump(Pump):
    def __init__(self, name: str, max_pressure_bar: float, max_flow_ml_s: float, em_lag_s: float):
        super().__init__(name)
        self.max_pressure_bar = max_pressure_bar
        self.max_flow_ml_s = max_flow_ml_s
        self.em_lag_s = em_lag_s
        self.current_power = 0.0
        
    def generate_fluid(self, dt: float, power_multiplier: float) -> Dict[str, FluidState]:
        lag_factor = min(1.0, dt / max(0.0001, self.em_lag_s))
        self.current_power += (power_multiplier - self.current_power) * lag_factor
        
        p_max = self.current_power * self.max_pressure_bar
        r_internal = self.max_pressure_bar / self.max_flow_ml_s if self.max_flow_ml_s > 0 else 0
        
        downstream_r = self.calculate_total_downstream_resistance()
        total_r = downstream_r + r_internal
        
        flow = p_max / total_r if total_r > 0 else 0.0
        pressure = flow * downstream_r
        
        return self.distribute_flow(flow, pressure, 20.0) # Assume 20C water from tank

class RotaryVanePump(Pump):
    def __init__(self, name: str, max_pressure_bar: float, max_flow_ml_s: float):
        super().__init__(name)
        self.max_pressure_bar = max_pressure_bar
        self.max_flow_ml_s = max_flow_ml_s
        
    def generate_fluid(self, dt: float, power_multiplier: float) -> Dict[str, FluidState]:
        is_on = power_multiplier > 0.5
        p_max = self.max_pressure_bar if is_on else 0.0
        r_internal = 0.1
        
        downstream_r = self.calculate_total_downstream_resistance()
        total_r = downstream_r + r_internal
        
        flow = p_max / total_r if total_r > 0 else 0.0
        pressure = flow * downstream_r
        
        return self.distribute_flow(flow, pressure, 20.0)

class Pipe(HydraulicNode):
    def __init__(self, name: str, compliance_ml_per_bar: float, internal_resistance: float = 0.05):
        super().__init__(name)
        self.compliance = compliance_ml_per_bar
        self.resistance = internal_resistance
        self.sim_pressure = 0.0
        
    def get_internal_resistance(self) -> float:
        return self.resistance
        
    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        target_pressure = max(0.0, fluid_in.pressure_bar - (fluid_in.flow_rate_ml_s * self.resistance))
        
        if self.compliance > 0:
            ramp = 2.0 / self.compliance
            self.sim_pressure += (target_pressure - self.sim_pressure) * ramp * dt
            dp = target_pressure - self.sim_pressure
            flow_out = max(0.0, fluid_in.flow_rate_ml_s - (dp * self.compliance))
        else:
            self.sim_pressure = target_pressure
            flow_out = fluid_in.flow_rate_ml_s
            
        return self.distribute_flow(flow_out, self.sim_pressure, fluid_in.temperature_c)

class OverPressureValve(HydraulicNode):
    def __init__(self, name: str, limit_bar: float):
        super().__init__(name)
        self.limit = limit_bar
        
    def update_downstream_resistance(self) -> float:
        # OPV bypass resistance is dynamic (pressure-dependent). During backward sweep,
        # we assume bypass is closed, and handle the split explicitly in forward sweep.
        r_main = 1e6
        if "main" in self.outputs:
            r_main = self.outputs["main"].update_downstream_resistance()
            self.downstream_resistances["main"] = r_main
            
        self.downstream_resistances["bypass"] = 1e6 # Assume closed initially
        return r_main + 0.1 # Brass T-fitting internal resistance
        
    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        p_out = max(0.0, fluid_in.pressure_bar - (fluid_in.flow_rate_ml_s * 0.1))
        flow_main = fluid_in.flow_rate_ml_s
        flow_bypass = 0.0
        
        if self.limit and p_out > self.limit:
            p_out = self.limit
            r_main = self.downstream_resistances.get("main", 1e6)
            flow_main = p_out / max(1e-6, r_main)
            flow_bypass = max(0.0, fluid_in.flow_rate_ml_s - flow_main)

        out = {}
        if "main" in self.outputs:
            out["main"] = FluidState(p_out, flow_main, fluid_in.temperature_c)
        if "bypass" in self.outputs:
            out["bypass"] = FluidState(0.0, flow_bypass, fluid_in.temperature_c) # Drops to atm
            
        return out

class SolenoidValve(HydraulicNode):
    def __init__(self, name: str, is_3_way: bool = False):
        super().__init__(name)
        self.is_open = False
        self.is_3_way = is_3_way
        
    def apply_logic(self, is_open: bool):
        self.is_open = is_open
        
    def get_internal_resistance(self) -> float:
        return 0.01
        
    def update_downstream_resistance(self) -> float:
        if "main" in self.outputs:
            self.downstream_resistances["main"] = self.outputs["main"].update_downstream_resistance()
        if "drain" in self.outputs:
            self.downstream_resistances["drain"] = self.outputs["drain"].update_downstream_resistance()
            
        if self.is_open:
            r_down = self.downstream_resistances.get("main", 1e6)
        else:
            r_down = self.downstream_resistances.get("drain", 1e6) if self.is_3_way else 1e6
        
        return self.get_internal_resistance() + r_down
        
    def process_fluid(self, dt: float, fluid_in: FluidState) -> Dict[str, FluidState]:
        r_int = self.get_internal_resistance()
        p_out = max(0.0, fluid_in.pressure_bar - (fluid_in.flow_rate_ml_s * r_int))
        
        out = {}
        if self.is_open:
            if "main" in self.outputs:
                out["main"] = FluidState(p_out, fluid_in.flow_rate_ml_s, fluid_in.temperature_c)
            if "drain" in self.outputs:
                out["drain"] = FluidState(0.0, 0.0, fluid_in.temperature_c)
        else:
            if "main" in self.outputs:
                out["main"] = FluidState(0.0, 0.0, fluid_in.temperature_c)
            if self.is_3_way and "drain" in self.outputs:
                out["drain"] = FluidState(p_out, fluid_in.flow_rate_ml_s, fluid_in.temperature_c)
                
        return out

class Drain(HydraulicNode):
    """Terminal node representing atmospheric venting (drip tray or cup)."""
    def __init__(self, name: str):
        super().__init__(name)
        self.total_volume_ml = 0.0

    def update_downstream_resistance(self) -> float:
        return 0.001

    def process_fluid(self, dt: float, fluid_in: FluidState):
        """Accumulates the liquid poured into the drain."""
        self.total_volume_ml += fluid_in.flow_rate_ml_s * dt
        return {} # No downstream flow

class HydraulicTopology:
    def __init__(self, pump: Pump):
        self.pump = pump
        self.links: Dict[str, FluidState] = {}
        self.nodes_topological_order: List[HydraulicNode] = self._build_topological_order()
        
    def _build_topological_order(self) -> List[HydraulicNode]:
        """Sorts the graph using BFS starting from the pump."""
        order = []
        visited = set()
        queue = [self.pump]
        
        while queue:
            curr = queue.pop(0)
            if curr.name not in visited:
                order.append(curr)
                visited.add(curr.name)
                for nxt in curr.outputs.values():
                    if nxt.name not in visited:
                        queue.append(nxt)
        return order

    def step(self, dt: float, pump_power: float) -> Dict[str, FluidState]:
        # 1. Backward Sweep (Calculate total path resistance recursively)
        self.pump.update_downstream_resistance()
        
        # 2. Source Generation
        fluids = self.pump.generate_fluid(dt, pump_power)
        
        # 3. Forward Sweep (Iterate graph and pass fluid packets)
        inbox: Dict[str, FluidState] = {self.pump.name: FluidState()} 
        
        for node in self.nodes_topological_order:
            if node == self.pump:
                out_fluids = fluids
            else:
                fluid_in = inbox.get(node.name, FluidState())
                out_fluids = node.process_fluid(dt, fluid_in)
            # Route output fluids to downstream nodes via named ports
            for port, nxt_node in node.outputs.items():
                link_name = f"{node.name}_{port}"
                self.links[link_name] = out_fluids[port]
                inbox[nxt_node.name] = out_fluids[port].clone()
                
        return self.links