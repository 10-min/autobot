class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, output_limits=(0,100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output, self.max_output = output_limits

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        output = max(self.min_output, min(self.max_output, output))

        self.prev_error = error
        
        return output
