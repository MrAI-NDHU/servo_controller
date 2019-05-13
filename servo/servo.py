class Servo:
    def __init__(self, angle_min: float, angle_max: float, pwm_val_min: float,
                 pwm_val_max: float, pwm_freq: float, sec_per_angle: float):
        self._angle_min = angle_min
        self._angle_max = angle_max
        self._pwm_val_min = pwm_val_min
        self._pwm_val_max = pwm_val_max
        self._pwm_freq = pwm_freq
        self._sec_per_angle = sec_per_angle
    
    @property
    def angle_min(self) -> float:
        return self._angle_min
    
    @property
    def angle_max(self) -> float:
        return self._angle_max
    
    @property
    def pwm_val_min(self) -> float:
        return self._pwm_val_min
    
    @property
    def pwm_val_max(self) -> float:
        return self._pwm_val_max
    
    @property
    def pwm_freq(self) -> float:
        return self._pwm_freq
    
    @property
    def sec_per_angle(self) -> float:
        return self._sec_per_angle
    
    def fix_angle(self, angle: float) -> float:
        if angle < self.angle_min:
            return self.angle_min
        elif angle > self.angle_max:
            return self.angle_max
        else:
            return angle
    
    def wait_time(self, rel_angle: float) -> float:
        return self.sec_per_angle * self.fix_angle(abs(rel_angle))
    
    def angle_to_pwm_val(self, angle: float) -> float:
        return self.pwm_val_min \
               + (self.fix_angle(angle) - self.angle_min) \
               / (self.angle_max - self.angle_min) \
               * (self.pwm_val_max - self.pwm_val_min)
