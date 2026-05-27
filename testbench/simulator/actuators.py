from simulator.core import EnvironmentState
import math

class SolidStateRelay:
    def __init__(self, pwm_period_s: float = 1.0):
        self.pwm_period_s = pwm_period_s
        self.duty_cycle = 0.0

    def apply_logic(self, logical_signal: float):
        """Accepts a PWM duty cycle (0.0 to 1.0) from the control core."""
        self.duty_cycle = max(0.0, min(1.0, logical_signal))

    def get_output_state(self, env: EnvironmentState) -> bool:
        """Returns True if the relay is currently conducting based on PWM period."""
        if self.duty_cycle <= 0.0:
            return False
        if self.duty_cycle >= 1.0:
            return True
            
        # Simulate low-frequency PWM time-proportioning
        position_in_period = env.current_time_s % self.pwm_period_s
        return position_in_period < (self.duty_cycle * self.pwm_period_s)


class Triac:
    def __init__(self):
        self.delay_us = 0

    def apply_delay(self, delay_us: int):
        """Applies the phase-angle firing delay calculated by the core."""
        self.delay_us = max(0, delay_us)

    def get_rms_voltage_multiplier(self, env: EnvironmentState) -> float:
        """Returns the effective RMS voltage multiplier (0.0 to 1.0) based on phase cut."""
        # Calculate the duration of an AC half-period in microseconds (e.g. 8333us for 60Hz)
        half_period_us = (1.0 / env.mains_frequency_hz) / 2.0 * 1_000_000
        
        if self.delay_us >= half_period_us:
            return 0.0
        if self.delay_us <= 0:
            return 1.0
            
        # Convert delay to a firing angle in radians (0 to Pi)
        alpha = math.pi * (self.delay_us / half_period_us)
        
        # Mathematics for RMS voltage of a phase-cut sine wave
        # Vrms = Vpk * sqrt( 1/2 - alpha/(2*pi) + sin(2*alpha)/(4*pi) )
        # Since we want a multiplier (0 to 1), we normalize it.
        term = 1.0 - (alpha / math.pi) + (math.sin(2.0 * alpha) / (2.0 * math.pi))
        return math.sqrt(max(0.0, term))


class HeatingElement:
    def __init__(self, max_power_w: float):
        self.max_power_w = max_power_w

    def get_applied_power_w(self, ssr_state: bool, env: EnvironmentState) -> float:
        """Calculates actual heat output in Watts, considering mains voltage sags."""
        if not ssr_state:
            return 0.0
            
        # Power is proportional to the square of the voltage.
        # If mains voltage sags (e.g., from 120V to 110V), the heater outputs less power.
        nominal_voltage = 120.0 # Assuming US nominal for the element spec
        voltage_ratio = env.mains_voltage_ac / nominal_voltage
        
        return self.max_power_w * (voltage_ratio ** 2)