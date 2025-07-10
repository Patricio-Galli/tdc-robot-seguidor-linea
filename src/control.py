# Constants
## Scan
DT = 0.1                                    # (ms)      # Defines how much virtual time advances per simulation cycle      

## Context
FLOOR_DAMPING = 0.5                         # (kg/s) 
LINE_WIDTH = 1.8                            # (cm)
PX_CM_RELATION = 6.83                       # (px/cm)
V_CM_RELATION = 3                           # (V/cm)

## Controller
PROPORTIONAL_THRESHOLD = 1 * PX_CM_RELATION # (px)
MIN_DERIVATIVE_SLOPE = 4 * PX_CM_RELATION      

## Robot
ROBOT_MASS = 1.0                            # (kg)  

## History
POINTS_OF_HISTORY = 500                                 # Number of data points to show on the graphs

# reference (theta_i) -> px
# error (e) -> px
# pid_output -> volts
# response (theta_0) -> cm
# light_perturbation (Pl) -> cm
# movement_perturbation (Pm) -> cm
# camera_output -> px

class PIDController:
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        # Proportional
        if abs(error) > PROPORTIONAL_THRESHOLD:
            p_term = self.Kp * error
        else:    
            p_term = 0

        # Integral
        self.integral += error * DT
        i_term = self.Ki * self.integral

        # Derivative
        if abs(error - self.previous_error) > MIN_DERIVATIVE_SLOPE:
            derivative = (error - self.previous_error) / DT
        else: 
            derivative = 0
        d_term = self.Kd * derivative
        self.previous_error = error

        return p_term + i_term + d_term, p_term, i_term, d_term

    def reset_gains(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # Reset state when gains change to avoid instability
        self.integral = 0.0
        self.previous_error = 0.0

class RobotModel:
    def __init__(self):
        self.x_axis_position = 0.0 # At the center of the line
        self.x_axis_velocity = 0.0 # Moves straight ahead

    def update(self, motor_voltage): 
        # Positive means the motor pushes to the right
        # Negative means the motor pushes to the left
        x_axis_acceleration = (motor_voltage - FLOOR_DAMPING * self.x_axis_velocity) / ROBOT_MASS
        self.x_axis_velocity += x_axis_acceleration * DT
        self.x_axis_position += self.x_axis_velocity * DT
        return self.x_axis_position

class Simulation:
    def __init__(self, kp, kd, ki, reference):
        self.pid = PIDController(kp, kd, ki)
        self.robot = RobotModel()
        self.history = History()
        self.reference = reference
        self.time = 0.0
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

    def update(self):
        # 1. Light perturbation (Pl(t)) affects the sensor reading.
        # This creates the error signal fed into the controller.
        feedback_signal = (self.robot.x_axis_position + self.light_perturbation)*PX_CM_RELATION
        error = self.reference - feedback_signal

        # 2. PID controller calculates the required motor voltage.
        pid_output, p, i, d = self.pid.update(error)
        motor_voltage = pid_output/PX_CM_RELATION * V_CM_RELATION

        # 3. The motor acts on the robot.
        self.robot.update(motor_voltage)

        # 4. Movement perturbation (Pf(t)/Pi(t)) acts as a direct physical push on the robot.
        self.robot.x_axis_position += self.movement_perturbation
        response = self.robot.x_axis_position # The final response includes this push

        # 5. Log data for plotting.
        total_pert = self.light_perturbation + self.movement_perturbation
        self.history.log(self.time, error, p, i, d, response, total_pert, self.reference, feedback_signal, self.robot.x_axis_velocity)

        self.time += DT

        # 6. Reset perturbations (they are Dirac-like, lasting one step).
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

        return self.history.get()

    def inject_light_perturbation(self, amplitude):
        self.light_perturbation = amplitude

    def inject_movement_perturbation(self, amplitude):
        self.movement_perturbation = amplitude

    def reset(self):
        self.time = 0.0
        self.robot = RobotModel()
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

        # Clean history
        self.history.reset()

class History:
    def __init__(self):
        self.data = {
            'time': [],
            'error': [],
            'p_out': [], 'i_out': [], 'd_out': [],
            'response': [],
            'total_pert': [],
            'reference': [],
            'feedback_signal': [],
            'velocity': [],
        }

    def log(self, time, error, p, i, d, response, total_pert, reference, feedback_signal, velocity):
        values = [time, error, p, i, d, response, total_pert, reference, feedback_signal, velocity]
        for key, val in zip(self.data.keys(), values):
            self.data[key].append(val)
            if len(self.data[key]) > POINTS_OF_HISTORY:
                self.data[key].pop(0)

    def reset(self):
        for key in self.data:
            self.data[key].clear()

    def get(self):
        return self.data