import random
from typing import Callable, Optional

class Sensor:
    def __init__(self, noise_std_dev: float, lag_s: float):
        self.noise_std_dev = noise_std_dev
        self.lag_s = max(0.0001, lag_s)  # Prevent division by zero
        self.internal_value = 0.0
        self.initialized = False
        self.read_source: Optional[Callable[[], float]] = None

    def attach(self, source: Callable[[], float]):
        """Attaches the sensor to a specific property in the simulation topology."""
        self.read_source = source

    def step(self, dt: float):
        """Fetches the attached property and applies a low-pass filter for latency."""
        if not self.read_source:
            return
            
        true_value = self.read_source()
        
        if not self.initialized:
            self.internal_value = true_value
            self.initialized = True
            
        # RC Low-pass filter formulation
        alpha = dt / (self.lag_s + dt)
        self.internal_value += alpha * (true_value - self.internal_value)

    def read(self) -> float:
        """Returns the delayed value with injected Gaussian noise."""
        noise = random.gauss(0.0, self.noise_std_dev)
        return self.internal_value + noise


class TemperatureSensor(Sensor):
    def __init__(self, noise_std_dev: float = 0.15, lag_s: float = 0.8):
        super().__init__(noise_std_dev, lag_s)


class PressureTransducer(Sensor):
    def __init__(self, noise_std_dev: float = 0.2, lag_s: float = 0.05):
        # Pressure sensors are generally faster than temp sensors but suffer from pump vibration
        super().__init__(noise_std_dev, lag_s)


class Scale(Sensor):
    def __init__(self, noise_std_dev: float = 0.5, lag_s: float = 0.2):
        super().__init__(noise_std_dev, lag_s)


class VolumetricFlowmeter:
    def __init__(self, pulses_per_ml: float):
        self.pulses_per_ml = pulses_per_ml
        self.accumulated_ml = 0.0
        self.pulse_count = 0
        self.read_source: Optional[Callable[[], float]] = None

    def attach(self, source: Callable[[], float]):
        """Attaches the flowmeter to a specific flow rate source."""
        self.read_source = source

    def step(self, dt: float):
        if not self.read_source:
            return
            
        flow_rate_ml_s = self.read_source()
        self.accumulated_ml += flow_rate_ml_s * dt

    def read_pulses(self) -> int:
        """Returns the number of digital ticks since the last read."""
        expected_total_pulses = int(self.accumulated_ml * self.pulses_per_ml)
        new_pulses_since_last_read = expected_total_pulses - self.pulse_count
        self.pulse_count = expected_total_pulses
        return new_pulses_since_last_read