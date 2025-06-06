from gpiozero import Button, PWMOutputDevice
import threading

class Servo:
    def __init__(self):
        # Encoder settings
        self.ENC_R = Button(10, pull_up=True)
        self.ENC_L = Button(2, pull_up=True)
        self.count_R = 0
        self.count_L = 0

        # Bind encoder callbacks to instance methods
        self.ENC_R.when_pressed = self.enc_callback_R
        self.ENC_R.when_released = self.enc_callback_R
        self.ENC_L.when_pressed = self.enc_callback_L
        self.ENC_L.when_released = self.enc_callback_L

        # Motor settings
        self.MOT_R_1 = PWMOutputDevice(pin=23, frequency=60)
        self.MOT_R_2 = PWMOutputDevice(pin=22, frequency=60)
        self.MOT_L_1 = PWMOutputDevice(pin=18, frequency=60)
        self.MOT_L_2 = PWMOutputDevice(pin=17, frequency=60)

        # Control parameters
        self.DURATION = 0.1  # Control interval (seconds)
        self.prev_count_R = 0
        self.prev_count_L = 0
        self.err_prev_R = 0
        self.err_prev_L = 0
        self.err_I_R = 0
        self.err_I_L = 0
        self.Kp = 20
        self.Ki = 100
        self.Kd = 0.1
        self.target_speed_R = 0.0
        self.target_speed_L = 0.0

    def enc_callback_R(self):
        if self.target_speed_R > 0:
            self.count_R += 1
        else:
            self.count_R -= 1

    def enc_callback_L(self):
        if self.target_speed_L > 0:
            self.count_L += 1
        else:
            self.count_L -= 1

    def init_variables_R(self):
        self.count_R = 0
        self.prev_count_R = 0
        self.err_prev_R = 0
        self.err_I_R = 0
        self.MOT_R_1.value = 0
        self.MOT_R_2.value = 0
    
    def init_variables_L(self):
        self.count_L = 0
        self.prev_count_L = 0
        self.err_prev_L = 0
        self.err_I_L = 0
        self.MOT_L_1.value = 0
        self.MOT_L_2.value = 0

    def drive(self):
        # --- Right Motor PID ---
        speed_R = (self.count_R - self.prev_count_R) / 40 / self.DURATION
        err_P = self.target_speed_R - speed_R
        self.err_I_R += err_P * self.DURATION
        err_D = (err_P - self.err_prev_R) / self.DURATION
        duty_R = self.Kp * err_P + self.Ki * self.err_I_R + self.Kd * err_D

        duty_R = max(min(duty_R, 100.0), -100.0)

        if duty_R > 0:
            self.MOT_R_1.value = duty_R / 100.0
            self.MOT_R_2.value = 0
        else:
            self.MOT_R_1.value = 0
            self.MOT_R_2.value = -duty_R / 100.0

        self.prev_count_R = self.count_R
        self.err_prev_R = err_P

        # --- Left Motor PID ---
        speed_L = (self.count_L - self.prev_count_L) / 40 / self.DURATION
        err_P = self.target_speed_L - speed_L
        self.err_I_L += err_P * self.DURATION
        err_D = (err_P - self.err_prev_L) / self.DURATION
        duty_L = self.Kp * err_P + self.Ki * self.err_I_L + self.Kd * err_D

        duty_L = max(min(duty_L, 100.0), -100.0)

        if duty_L > 0:
            self.MOT_L_1.value = duty_L / 100.0
            self.MOT_L_2.value = 0
        else:
            self.MOT_L_1.value = 0
            self.MOT_L_2.value = -duty_L / 100.0

        self.prev_count_L = self.count_L
        self.err_prev_L = err_P

        # Schedule the next control update
        t = threading.Timer(self.DURATION, self.drive)
        t.start()

    def set_speed(self, speed_L, speed_R):
        self.target_speed_L = speed_L
        if self.target_speed_L == 0:
            self.init_variables_L()
        self.target_speed_R = speed_R
        if self.target_speed_R == 0:
            self.init_variables_R()

def main():
    servo = Servo()
    servo.drive()  # Initial call

    try:
        while True:
            pass
    except KeyboardInterrupt:
        # Stop all motors on exit
        servo.MOT_R_1.value = 0
        servo.MOT_R_2.value = 0
        servo.MOT_L_1.value = 0
        servo.MOT_L_2.value = 0

if __name__ == "__main__":
    main()
