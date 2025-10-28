import RPi.GPIO as GPIO
import time
import threading

A = 12
B = 13
IN1 = 22
IN2 = 25
IN3 = 5
IN4 = 6

L_ENCODER = 17
L_ENCODER2 = 27
R_ENCODER = 23
R_ENCODER2 = 24

class Motor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A, GPIO.OUT)
        GPIO.setup(B, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(L_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(R_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(L_ENCODER2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(R_ENCODER2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.PWM_A = GPIO.PWM(A, 50)
        self.PWM_B = GPIO.PWM(B, 50)
        
        self.l_direction = 0
        self.r_direction = 0
        
        self.left_count = 0
        self.right_count = 0
        
        self.l_rpm = 0.0
        self.r_rpm = 0.0
        
        self.start_time = time.time()
        
        GPIO.add_event_detect(L_ENCODER, GPIO.RISING, callback=self.count_la)
        GPIO.add_event_detect(R_ENCODER, GPIO.RISING, callback=self.count_ra)
        
        self.start()
    def start(self):
        self.PWM_A.start(0.0)
        self.PWM_B.start(0.0)
        
    def stop(self):
        self.PWM_A.stop()
        self.PWM_B.stop()
        
    def set_l_motor(self, pwm):
        if pwm > 0:
            GPIO.output(IN1, False)
            GPIO.output(IN2, True)
            self.PWM_A.ChangeDutyCycle(min(100, pwm))
            self.l_direction = 1
        else:
            GPIO.output(IN1, True)
            GPIO.output(IN2, False)
            self.PWM_A.ChangeDutyCycle(min(100, abs(pwm)))
            self.l_direction = -1
            
    def set_r_motor(self, pwm):
        if pwm > 0:
            GPIO.output(IN3, True)
            GPIO.output(IN4, False)
            self.PWM_B.ChangeDutyCycle(min(100, pwm))
            self.r_direction = 1
        else:
            GPIO.output(IN3, False)
            GPIO.output(IN4, True)
            self.PWM_B.ChangeDutyCycle(min(100, abs(pwm)))
            self.r_direction = -1

    def __del__(self):
        self.PWM_A.stop()
        self.PWM_B.stop()
        GPIO.cleanup()
        
    def count_la(self, channel):
        self.left_count += 1 * self.l_direction
        #print("l: ", self.left_count)
        
    def count_ra(self, channel):
        self.right_count += 1 * self.r_direction
        #print("r: ", self.right_count)
            
    def get_encoder_counts(self):
        return self.left_count, self.right_count
