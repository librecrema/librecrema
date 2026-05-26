LibreCrema
==========

> [!WARNING]
> LibreCrema is, for now, a purely theoretical implementation and is yet to be deployed on real, physical hardware. This is a personal hobby project currently under active development. The ultimate goal is to evolve this codebase until it is stable and robust enough to safely and reliably drive basic physical espresso machines.

LibreCrema is a modular, hardware-agnostic, and safe control engine designed for next-generation espresso machines. Built in Rust with a strict separation between an idiomatic safe core and a thin FFI/C translation layer, it achieves 100% memory safety, predictable execution times, and zero heap allocation—making it ideal for bare-metal deployment on microcontrollers like the ESP32.

Key Architectural Advantages
----------------------------

*   **Trait-Based Architecture:** Logic blocks (PIDs, Thermostats, Watchdogs, Sequencers) implement a native, strongly typed `Node` trait.
    
*   **Static Dispatch Engine:** Avoids runtime dynamic dispatch vtables (`dyn Trait`) and memory allocations. The scheduler utilizes a unified, statically sized `CoreNode` enum, yielding deterministic execution steps.
    
*   **Decoupled FFI Layer (`ffi.rs`):** Raw pointers, C-style unions, and platform-specific structure alignments are strictly isolated to the boundary. The internal control loops remain 100% safe, idiomatic Rust.
    
*   **Dual-Mode Target Compilation:** Built as a pure, lightweight `#![no_std]` crate by default for embedded microcontrollers, but seamlessly compiles standard-library hooks (`with-std`) on host platforms for unit testing and physical simulator testbenches.
    

System Architecture Overview
----------------------------

                            +---------------------------------------------+
                            |      Python Simulator / C/C++ ESP-IDF       |
                            +---------------------------------------------+
                                                   |
                                                   | (C ABI Pointer Boundary)
                                                   v
                            +---------------------------------------------+
                            |             FFI Adaptor Layer               |
                            |      (Translates PortC <-> PortValue)       |
                            +---------------------------------------------+
                                                   |
                                                   | (Safe Static Dispatch)
                                                   v
                            +---------------------------------------------+
                            |               Rust Safe Core                |
                            |       - SafeScheduler                       |
                            |       - CoreNode (PID, PAM, Sequencer...)   |
                            +---------------------------------------------+
    

### Supported Control Nodes

1.  **PID Controller (`PidController`):** Equipped with predictive anti-windup clamping to prevent integral windup, a high-fidelity low-pass filter on the derivative term to suppress noise, and a bumpless manual-to-auto transfer tracking mode.
    
2.  **Phase Angle Mapper (`PhaseAngleMapper`):** Solves the transcendental AC root-mean-square (RMS) power equation using a binary search solver to calculate precise microsecond TRIAC trigger delays.
    
3.  **Matrix Sequencer (`Sequencer`):** A state machine that guides the machine through customizable pressure and temperature stages (Pre-Infusion, Ramp, Extraction, Decline, and Standby pre-heating at 93.5∘C).
    
4.  **Safety Watchdog (`SafetyWatchdog`):** A protective supervisor that monitors temperature and pressure boundaries, triggering immediate hardware shutdowns if unsafe limits are breached for more than a defined timeout window.
    
5.  **Hysteresis Thermostat (`Thermostat`):** A reliable deadband-regulated temperature switch.
    
6.  **Virtual Hydraulic Sensors:** Mocks fluid dynamics using Darcy's Law and system power variables.
    

🛠️ Rust Library Development
----------------------------

### 1\. Host Target Testing

Unit and integration tests run in standard host-mode. To run tests on your local machine with standard library features active:

    cargo test
    

### 2\. Bare-Metal Build (Embedded)

To compile a lightweight `#![no_std]` target library containing the bare-metal abort panic handlers for target microcontrollers:

    cargo build --release --no-default-features
    

### 3\. Automatic Header Generation

Whenever you build the library on your PC, `build.rs` will automatically invoke `cbindgen` to parse `src/ffi.rs` and write/update the C-compatible header interface (`librecrema.h`) directly alongside your compiled binary inside the `target/<profile>/` directory.

💻 Physics Simulation Testbench (`/simulator`)
----------------------------------------------

The `/simulator` directory contains a high-fidelity Software-in-the-Loop (SIL) simulation environment. It models a modern **1600W espresso machine** equipped with a low-mass thermoblock and non-linear porous-media hydraulics.

The simulation accurately captures:

*   **1600W Thermoblock Transient Thermodynamics:** Multi-stage heating, convective losses, and cold water advection thermal load drops.
    
*   **Darcy's Law Coffee Puck Hydraulics:** Compressible porous media flow modeling, dynamic grounds swelling, and puck erosion.
    
*   **Physical Actuator Inertia:** Models the electromagnetic lag (τpump​\=150ms) of a vibratory pump and water line hydraulic capacitance (τhydraulic​\=200ms) to prevent digital feedback chatter.
    
*   **Closed-Loop Cascade Control:** Real-time PID temperature and pressure tracking.
    

### Dependencies

Install the Python dependencies on your host machine:

    pip install -r simulator/requirements.txt
    

### Running the Simulator

To automatically trigger the Rust compiler, build the `.so`/`.dylib` bindings, and run the 60-second simulated extraction:

    python simulator/run_simulation.py
    

_The run will output real-time console metrics and generate an analytical chart saved as `librecrema_simulation_telemetry.png`._

🔌 ESP32 Hardware Transition Guide
----------------------------------

Transitioning from the Python simulator to physical hardware involves linking the static library (`liblibrecrema.a`) to an ESP-IDF C++ project and mapping virtual inputs/outputs to real hardware interfaces.

### 1\. C FFI Data Mapping

Every 10ms inside your primary control loop, poll your hardware peripherals and inject the measurements into the scheduler's input ports:

    #include "librecrema.h"
    
    void app_main() {
        // Initialize Scheduler
        librecrema_init();
        
        // Register physical hardware nodes
        librecrema_add_pid_node(NODE_ID_PID, 35.0f, 0.35f, 4.5f, 0.0f, 100.0f, 0.8f);
        librecrema_add_phase_angle_node(NODE_ID_PHASE_ANGLE, 60, 7500);
        librecrema_add_pressure_pid_node(NODE_ID_PRESSURE_PID, 0.08f, 0.06f, 0.005f, 0.0f, 1.0f, 0.15f);
        
        // Establish topological bindings
        librecrema_add_binding(NODE_ID_SEQUENCER, 1, NODE_ID_PID, 0);          // Seq Target Temp -> Temp PID Setpoint
        librecrema_add_binding(NODE_ID_SEQUENCER, 0, NODE_ID_PRESSURE_PID, 0); // Seq Target Press -> Press PID Setpoint
        librecrema_add_binding(NODE_ID_PRESSURE_PID, 0, NODE_ID_PHASE_ANGLE, 0); // Press PID Output -> Pump TRIAC Power
    
        while (1) {
            // Read physical sensors
            float actual_temp = read_max6675_thermocouple();
            float actual_pressure = read_analog_pressure_transducer();
            
            // Map to FFI structures
            PortC temp_port = { Float, { .f_val = actual_temp } };
            PortC press_port = { Float, { .f_val = actual_pressure } };
            
            // Inject into scheduler
            librecrema_set_port(NODE_ID_PID, 1, &temp_port);
            librecrema_set_port(NODE_ID_PRESSURE_PID, 1, &press_port);
            
            // Step the control engine
            librecrema_tick(10); // 10ms Step
            
            // Drive physical actuators based on computed results
            PortC heater_power;
            librecrema_get_port(NODE_ID_PID, 0, &heater_power);
            set_thermoblock_solid_state_relay(heater_power.value.f_val); // Low-frequency SSR PWM
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    

### 2\. High-Frequency TRIAC Phase Actuation

To actuate a typical AC vibratory pump (e.g., Ulka EP5) safely, synchronize your TRIAC triggers with AC zero-crossings using the ESP32 hardware interrupt (GPIO ISR) and a hardware timer:

1.  **Zero-Cross Interrupt:** Connect an isolated zero-crossing detector (e.g., H11AA1) to a GPIO pin. Trigger a high-priority ISR on every rising edge (every 8.33ms on 60Hz mains).
    
2.  **Delay Timer Scheduling:** Inside the ISR, fetch the latest delay output from the `PhaseAngleMapper` and schedule an ESP32 hardware timer (`gptimer`) to trigger after that delay:
    
        static void IRAM_ATTR zero_cross_isr_handler(void* arg) {
            PortC delay_port;
            librecrema_get_port(NODE_ID_PHASE_ANGLE, 0, &delay_port);
            uint32_t delay_us = delay_port.value.i_val;
        
            if (delay_us == 0) {
                gpio_set_level(TRIAC_GATE_PIN, 1); // Trigger immediately (Full power)
            } else if (delay_us < 8333) {
                esp_timer_start_once(pump_triac_timer, delay_us); // Fire after delay
            }
        }
        
    
3.  **TRIAC Gate Pulse:** When the scheduled timer expires, pull the TRIAC Gate Pin high for a short 15 μs pulse to latch the TRIAC open, allowing current to drive the pump coil for the remainder of the AC half-cycle.
    

📜 License
----------

LibreCrema is licensed under the Apache License, Version 2.0. See the `LICENSE` file for details.
