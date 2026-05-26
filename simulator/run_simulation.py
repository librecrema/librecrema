import os
import sys
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from cffi import FFI

# Initialize Python FFI context
ffi = FFI()

# Core Node IDs used within our topological graph mapping configurations
NODE_ID_SEQUENCER = 10
NODE_ID_PID = 20
NODE_ID_PHASE_ANGLE = 30
NODE_ID_SAFETY = 40
NODE_ID_PRESSURE_PID = 50 # Dedicated Pressure Closed-Loop control node

def compile_rust():
    """
    Safely triggers a release rebuild of the Rust static/shared library 
    dependencies directly from python prior to initializing the bindings.
    """
    print("Compiling Rust Core library with std enabled for Python FFI support...")
    rust_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    try:
        subprocess.run(["cargo", "build", "--release"], cwd=rust_dir, check=True)
        print("Rust core library compilation completed successfully!")
    except Exception as e:
        print(f"Error compiling Rust core library: {e}")
        sys.exit(1)

def load_ffi():
    """
    Parses the cbindgen header file, strips out preprocessor directives 
    unsupported by CFFI, and loads the platform-dependent shared object.
    """
    header_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../include/librecrema.h'))
    rust_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Locate platform-specific dynamic library binary
    if sys.platform.startswith('win'):
        lib_path = os.path.join(rust_dir, 'target/release/librecrema.dll')
    elif sys.platform.startswith('darwin'):
        lib_path = os.path.join(rust_dir, 'target/release/liblibrecrema.dylib')
    else:
        lib_path = os.path.join(rust_dir, 'target/release/liblibrecrema.so')

    if not os.path.exists(header_path):
        header_path = os.path.join(rust_dir, 'target/release/librecrema.h')

    if not os.path.exists(lib_path):
        raise FileNotFoundError(f"Could not locate the compiled shared library binary at: {lib_path}")

    # Read and clean C Header file for CFFI parsing compatibility
    with open(header_path, 'r') as f:
        header_content = f.read()

    clean_lines = []
    for line in header_content.splitlines():
        if line.strip().startswith('#'):
            continue
        clean_lines.append(line)
    
    clean_cdef = "\n".join(clean_lines)
    ffi.cdef(clean_cdef)
    
    return ffi.dlopen(lib_path)

# Thermodynamic state parameters (Optimized for modern low-mass thermoblocks)
C_block = 180.0       # Thermal capacity (J/K) for fast warm-up and high responsiveness
hA = 1.2             # Convective loss coefficient to ambient air (W/K)
T_ambient = 22.0     # Cold water inlet and environmental temperature (°C)
C_water = 4184.0     # Specific heat capacity of water (J/kg·K)
P_heater_max = 1600.0 # High-performance thermoblock heater maximum power (Watts)

# Darcy's porous media hydraulic coffee puck parameters
k_initial = 13      # Fresh puck initial hydraulic permeability constant (milli-Darcys)
alpha = 0.01         # Swelling/saturation rate of restriction per ml of passed water
beta = 0.005          # Elastic compressibility degradation coefficient per bar of pressure
headspace_fill_ml = 5.0 # Dead volume capacity of the empty group head cavity

def run_simulation(lib):
    """
    Runs a software-in-the-loop simulation of a rapid warm-up cycle 
    followed by a standard matrix-sequenced espresso extraction.
    """
    print("\nInitializing LibreCrema Scheduler...")
    lib.librecrema_init()

    # 1. Instantiate the Temperature PID regulation node
    # ID: 20, Kp=35.0, Ki=0.35, Kd=4.5, Min=0.0%, Max=100.0%, FilterCoeff=0.8
    lib.librecrema_add_pid_node(NODE_ID_PID, 35.0, 0.35, 4.5, 0.0, 100.0, 0.8)

    # 2. Instantiate Phase-Angle mapping node (Translates power requests to Triac timings)
    # ID: 30, Mains=60Hz, Max Delay=7500us
    lib.librecrema_add_phase_angle_node(NODE_ID_PHASE_ANGLE, 60, 7500)

    # 3. Instantiate the closed-loop Pressure PID control node
    # ID: 50, Kp=0.08, Ki=0.06, Kd=0.005, Min=0.0 (0% pump effort), Max=1.0 (100% pump effort)
    # FilterCoeff is lowered to 0.15 to heavily filter out discrete-time high-frequency derivative chatter.
    lib.librecrema_add_pid_node(NODE_ID_PRESSURE_PID, 0.08, 0.06, 0.005, 0.0, 1.0, 0.15)

    # 4. Instantiate the Matrix Sequencer Profiling State Machine
    # ID: 10, Pre-infusion, Ramp/Extraction, Decline stages
    stages = ffi.new("struct StageParameters[]", [
        {
            "stage": lib.PreInfusion,
            "duration_ms": 5000,
            "target_pressure_bar": 2.5,
            "target_temp_c": 93.5,
        },
        {
            "stage": lib.Extraction,
            "duration_ms": 20000,
            "target_pressure_bar": 9.0,
            "target_temp_c": 93.5,
        },
        {
            "stage": lib.Decline,
            "duration_ms": 5000,
            "target_pressure_bar": 5.0,
            "target_temp_c": 92.0,
        }
    ])
    lib.librecrema_add_sequencer_node(NODE_ID_SEQUENCER, stages, 3)

    # 5. Instantiate Safety watchdog limits node
    # ID: 40, MaxTemp=115°C, MaxPressure=12Bar, TimeoutLimit=2500ms
    lib.librecrema_add_safety_node(NODE_ID_SAFETY, 115.0, 12.0, 2500)

    # ========================================================
    # Dynamic Graph Signal Bindings (Configures machine topology)
    # ========================================================
    # Sequencer Target Temperature -> Temperature PID Setpoint (Input Port 0)
    lib.librecrema_add_binding(NODE_ID_SEQUENCER, 1, NODE_ID_PID, 0)

    # Sequencer Target Pressure -> Pressure PID Setpoint (Input Port 0)
    lib.librecrema_add_binding(NODE_ID_SEQUENCER, 0, NODE_ID_PRESSURE_PID, 0)

    # Pressure PID Output -> Phase Angle Input Power (Input Port 0)
    lib.librecrema_add_binding(NODE_ID_PRESSURE_PID, 0, NODE_ID_PHASE_ANGLE, 0)

    # Initialize FFI telemetry overrides using pointers to prevent ABI/libffi value issues
    def get_port_float(node, idx):
        port_c = ffi.new("struct PortC*")
        if lib.librecrema_get_port(node, idx, port_c):
            return port_c.value.f_val
        return 0.0

    def get_port_bool(node, idx):
        port_c = ffi.new("struct PortC*")
        if lib.librecrema_get_port(node, idx, port_c):
            return port_c.value.b_val
        return False

    def get_port_int(node, idx):
        port_c = ffi.new("struct PortC*")
        if lib.librecrema_get_port(node, idx, port_c):
            return port_c.value.i_val
        return 0

    def set_port_float(node, idx, val):
        port_c = ffi.new("struct PortC*")
        port_c.value_type = lib.Float
        port_c.value.f_val = val
        lib.librecrema_set_port(node, idx, port_c)

    def set_port_bool(node, idx, val):
        port_c = ffi.new("struct PortC*")
        port_c.value_type = lib.Bool
        port_c.value.b_val = val
        lib.librecrema_set_port(node, idx, port_c)

    # Simulation variables
    dt_ms = 10                  # Time interval step (10ms)
    total_time_s = 60           # Total simulation timeline (60 seconds)
    total_steps = int(total_time_s * 1000 / dt_ms)
    
    # Physical system states
    T_block = T_ambient
    accumulated_water_ml = 0.0
    puck_pressure_bar = 0.0
    actual_flow_rate_ml_min = 0.0
    pump_power_pct_buffered = 0.0 # Buffer state to track vibratory pump physical piston inertia

    # History buffers for plotting metrics
    history_time = []
    history_t_block = []
    history_t_setpoint = []
    history_p_puck = []
    history_p_target = []
    history_flow = []
    history_yield = []
    history_heater_pwr = []
    history_pump_delay = []

    print("Beginning 60s Simulation run...")
    print("Stage 1: Warming up Thermoblock to 93.5°C on-demand (t = 0s to 30s)...")
    
    # Active extraction trigger (will change to true at t = 30 seconds)
    is_brewing = False

    for step in range(total_steps):
        t_current = (step * dt_ms) / 1000.0
        
        # Trigger coffee brew at 30 seconds
        if t_current >= 30.0 and not is_brewing:
            is_brewing = True
            print("Stage 2: Starting Extraction Sequencer Profile (t = 30s)...")

        # Inject real-time sensor variables into input ports
        set_port_float(NODE_ID_PID, 1, T_block)                       # Thermoblock Temperature PV
        set_port_bool(NODE_ID_PID, 3, True)                           # Enable Auto PID Regs
        set_port_bool(NODE_ID_SEQUENCER, 0, is_brewing)               # Trigger brew profile sequencer
        set_port_float(NODE_ID_SAFETY, 0, T_block)                    # Safety monitor Temp
        set_port_float(NODE_ID_SAFETY, 1, puck_pressure_bar)           # Safety monitor Pressure
        set_port_float(NODE_ID_PRESSURE_PID, 1, puck_pressure_bar)     # Injected Puck Pressure feedback PV
        set_port_bool(NODE_ID_PRESSURE_PID, 3, is_brewing)             # Enable Pressure Loop only when brewing

        # Tick the topological execution engine once
        lib.librecrema_tick(dt_ms)

        # Retrieve computed actuator controls from target output ports via safe helpers
        heater_power_pct = get_port_float(NODE_ID_PID, 0) / 100.0
        target_pressure_bar = get_port_float(NODE_ID_SEQUENCER, 0)     # Target Pressure
        target_temp_c = get_port_float(NODE_ID_SEQUENCER, 1)           # Target Temperature
        
        # Apply safety overrides
        heater_tripped = get_port_bool(NODE_ID_SAFETY, 2)
        if heater_tripped:
            heater_power_pct = 0.0

        # Run thermodynamics math step (Lumped capacitance Integration)
        heat_in = heater_power_pct * P_heater_max
        heat_lost_convection = hA * (T_block - T_ambient)
        
        # Heat lost to advection as water flows through the channel
        mass_flow_rate_kg_s = (actual_flow_rate_ml_min / 60.0) / 1000.0 # Convert ml/min to kg/s
        heat_lost_advection = mass_flow_rate_kg_s * C_water * (T_block - T_ambient)
        
        dT_dt = (heat_in - heat_lost_convection - heat_lost_advection) / C_block
        T_block += dT_dt * (dt_ms / 1000.0)

        # Hydrology & Darcy's Law mechanical puck calculations
        if is_brewing and target_pressure_bar > 0.0:
            # Check Phase Angle output computed by the topological Pressure PID -> PAM path
            phase_delay_us = get_port_int(NODE_ID_PHASE_ANGLE, 0)
            
            # Map delay back to dynamic power delivery (0% corresponding to maximum delay 7500us)
            pump_power_pct = 1.0 - (phase_delay_us / 7500.0)
            pump_power_pct = max(0.0, min(1.0, pump_power_pct))
            
            # Apply first-order physical mechanical/electromagnetic lag of the pump piston (tau = 150ms)
            # This completely dampens discrete-time numerical resonance.
            tau_pump = 0.15 # seconds
            pump_power_pct_buffered += (pump_power_pct - pump_power_pct_buffered) * (dt_ms / 1000.0) / tau_pump
            
            # Darcy's porous resistance solver:
            # 1. Fill empty group headspace first
            if accumulated_water_ml < headspace_fill_ml:
                # Open hydraulic flow, minimal resistance
                p_steady_state = 0.0
                actual_flow_rate_ml_min = 350.0 * pump_power_pct_buffered # High volumetric free flow
            else:
                # Fluid hits puck: Exponential saturation and elastic pressure compression
                effective_yield = accumulated_water_ml - headspace_fill_ml
                permeability = k_initial * np.exp(-alpha * effective_yield) * (1.0 - beta * puck_pressure_bar)
                permeability = max(0.02, permeability) # Lower restriction limit
                
                # hydraulic resistance
                puck_resistance = 1.0 / permeability
                
                # Quadratic intersection solve against the non-linear Ulka EP5 physical pump curve:
                # P = P_max * (1 - (Q / Q_max)^2) and P = Q * R_puck
                P_max = 15.0     # Bar
                Q_max = 650.0    # ml/min
                
                a = P_max / (Q_max ** 2)
                b = puck_resistance
                c = -P_max * pump_power_pct_buffered
                
                # Analytical quadratic equation solver
                actual_flow_rate_ml_min = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
                actual_flow_rate_ml_min = max(0.0, actual_flow_rate_ml_min)
                p_steady_state = actual_flow_rate_ml_min * puck_resistance
            
            # Apply hydraulic capacitance time constant (200ms) to model real water line pressure dampening
            tau_hydraulic = 0.20 # seconds
            puck_pressure_bar += (p_steady_state - puck_pressure_bar) * (dt_ms / 1000.0) / tau_hydraulic
            accumulated_water_ml += (actual_flow_rate_ml_min / 60.0) * (dt_ms / 1000.0)
        else:
            puck_pressure_bar = 0.0
            actual_flow_rate_ml_min = 0.0
            phase_delay_us = 7500
            pump_power_pct_buffered = 0.0

        # Append step metrics to logs
        history_time.append(t_current)
        history_t_block.append(T_block)
        history_t_setpoint.append(target_temp_c)
        history_p_puck.append(puck_pressure_bar)
        history_p_target.append(target_pressure_bar)
        history_flow.append(actual_flow_rate_ml_min)
        history_yield.append(max(0.0, accumulated_water_ml - headspace_fill_ml))
        history_heater_pwr.append(heater_power_pct * 100)
        history_pump_delay.append(phase_delay_us)

    # ========================================================
    # Visual Analytics Generation
    # ========================================================
    print("\nExtraction complete! Generating charts...")
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('LibreCrema Hardware-in-the-Loop Simulation Analytics', fontsize=14, fontweight='bold')

    # Chart 1: Thermodynamic thermal performance
    axs[0, 0].plot(history_time, history_t_block, label='Thermoblock Temp (°C)', color='firebrick', linewidth=2)
    axs[0, 0].plot(history_time, history_t_setpoint, '--', label='Setpoint (°C)', color='gray')
    axs[0, 0].axvline(x=30.0, color='darkgreen', linestyle=':', label='Brew Triggered')
    axs[0, 0].set_title('Thermoblock Regulation (Warm-up -> Brew Advection)')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Temperature (°C)')
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # Chart 2: Hydraulic Pressure Profiling
    axs[0, 1].plot(history_time, history_p_puck, label='Simulated Puck Pressure', color='navy', linewidth=2)
    axs[0, 1].plot(history_time, history_p_target, '--', label='Target Profile', color='orange')
    axs[0, 1].axvline(x=30.0, color='darkgreen', linestyle=':')
    axs[0, 1].set_title('Dynamic Darcy-Ulka Pressure profiling')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Pressure (Bar)')
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    # Chart 3: Fluid Volumetrics
    ax3_y2 = axs[1, 0].twinx()
    p1 = axs[1, 0].plot(history_time, history_flow, label='Flow Rate (ml/min)', color='teal', linewidth=1.5)
    p2 = ax3_y2.plot(history_time, history_yield, label='Yield (ml)', color='purple', linewidth=2)
    axs[1, 0].axvline(x=30.0, color='darkgreen', linestyle=':')
    axs[1, 0].set_title('Extraction Volumetric Yields')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Flow Rate (ml/min)')
    ax3_y2.set_ylabel('Accumulated Yield (ml)')
    axs[1, 0].grid(True)
    plots = p1 + p2
    labels = [pl.get_label() for pl in plots]
    axs[1, 0].legend(plots, labels, loc='upper left')

    # Chart 4: Electrical Actuator Outputs
    ax4_y2 = axs[1, 1].twinx()
    p3 = axs[1, 1].plot(history_time, history_heater_pwr, label='Heater Duty Cycle (%)', color='orangered', linewidth=1.5)
    p4 = ax4_y2.plot(history_time, history_pump_delay, label='Pump Triac Delay (us)', color='goldenrod', linewidth=1.5)
    axs[1, 1].axvline(x=30.0, color='darkgreen', linestyle=':')
    axs[1, 1].set_title('Actuator Outputs (Low-Freq PWM & Phase-Angle delay)')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Heater Power (%)')
    ax4_y2.set_ylabel('Triac Delay (us)')
    axs[1, 1].grid(True)
    plots = p3 + p4
    labels = [pl.get_label() for pl in plots]
    axs[1, 1].legend(plots, labels, loc='upper left')

    plt.tight_layout()
    print("Saving telemetry chart to 'librecrema_simulation_telemetry.png'...")
    plt.savefig('librecrema_simulation_telemetry.png')
    plt.show()

if __name__ == "__main__":
    compile_rust()
    lib = load_ffi()
    run_simulation(lib)