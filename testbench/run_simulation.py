import matplotlib.pyplot as plt
import math
from simulator.orchestrator import SimulationOrchestrator

def main():
    profile_to_run = "default"
    
    # 1. Initialize the orchestrator
    orchestrator = SimulationOrchestrator(profile_name=profile_to_run, start_empty=False)
    
    # 2. Dynamically set logging points and assign them to groups
    # Group: Temperatures
    orchestrator.add_telemetry_point("Brew Boiler Temp", lambda: orchestrator.brew_boiler.temperature_c, group="Temperatures (°C)")
    orchestrator.add_telemetry_point("Group Metal Temp", lambda: orchestrator.group_head.temperature_c, group="Temperatures (°C)")
    orchestrator.add_telemetry_point("Sensor: Brew Temp", lambda: orchestrator.temp_sensor_brew.read(), group="Temperatures (°C)")
    
    if orchestrator.profile.has_steam_boiler:
        orchestrator.add_telemetry_point("Steam Boiler Temp", lambda: orchestrator.steam_boiler.temperature_c, group="Temperatures (°C)")
        
    # Group: Pressure
    orchestrator.add_telemetry_point("Sensor: Group Pressure", lambda: orchestrator.pressure_sensor.read(), group="Pressure (Bar)")
    
    # Group: Yield
    orchestrator.add_telemetry_point("Shot Yield", lambda: orchestrator.cup.total_volume_ml, group="Yield (ml)")
    orchestrator.add_telemetry_point("Shot Wastes", lambda: orchestrator.drip_tray.total_volume_ml, group="Yield (ml)")

    # Group: Control Signals
    orchestrator.add_telemetry_point("Brew Heater PWM", lambda: orchestrator.rust_core.current_brew_heater_pwm, group="Control Signals")
    if orchestrator.profile.has_steam_boiler:
        orchestrator.add_telemetry_point("Steam Heater PWM", lambda: orchestrator.rust_core.current_steam_heater_pwm, group="Control Signals")
    
    # 3. Run the simulation
    duration_s = 120.0
    telemetry, groups = orchestrator.run(duration_s)
    
    # 4. Dynamic Plotting
    print("Generating dynamic telemetry graphs...")
    
    # Auto-calculate grid layout (2 columns) based on the number of groups
    num_groups = len(groups)
    cols = 2
    rows = math.ceil(num_groups / cols)
    
    fig = plt.figure(figsize=(12, 3.5 * rows))
    
    for i, (group_name, metrics) in enumerate(groups.items()):
        ax = fig.add_subplot(rows, cols, i + 1)
        
        for metric in metrics:
            ax.plot(telemetry["time"], telemetry[metric], label=metric, linewidth=2)
            
            # Apply step styling for discrete states or PWM signals
            if "State" in metric or "PWM" in metric:
                ax.lines[-1].set_drawstyle("steps-post")
                
        ax.set_title(group_name)
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(loc="best")
        
        # Only add X-labels to the bottom row for cleaner aesthetics
        if i >= num_groups - cols:
            ax.set_xlabel("Time (Seconds)")
            
    plt.tight_layout()
    
    # Save and display
    output_filename = "librecrema_dynamic_telemetry.png"
    plt.savefig(output_filename, dpi=300)
    print(f"Success! Telemetry graph saved to '{output_filename}'.")
    plt.show()

if __name__ == "__main__":
    main()