import customtkinter as ctk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --- Constants ---
POINTS_OF_HISTORY = 200  # Number of data points to show on the graphs

# --- Simulation Components ---

class PIDController:
    """Implements a PID controller."""
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        p_term = self.Kp * error
        self.integral += error * self.dt
        i_term = self.Ki * self.integral
        derivative = (error - self.previous_error) / self.dt
        d_term = self.Kd * derivative
        self.previous_error = error
        return p_term + i_term + d_term, p_term, i_term, d_term

    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # Reset state when gains change to avoid instability
        self.integral = 0
        self.previous_error = 0

class RobotModel:
    """Simulates the robot's physical dynamics (the 'plant')."""
    def __init__(self, dt):
        self.dt = dt
        self.position = 0.0  # theta_o
        self.velocity = 0.0
        self.mass = 1.0
        self.damping = 0.5

    def update(self, motor_voltage):
        acceleration = (motor_voltage - self.damping * self.velocity) / self.mass
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt
        return self.position

class Simulation:
    """Manages the simulation based on the block diagram."""
    def __init__(self, dt=0.1):
        self.dt = dt
        self.time = 0.0
        self.pid = PIDController(1.0, 0.1, 0.01, self.dt)
        self.robot = RobotModel(self.dt)
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0
        self.history = {k: [] for k in ['time', 'error', 'p_out', 'i_out', 'd_out', 'response', 'total_pert']}

    def update(self):
        # 1. Light perturbation (Pl(t)) affects the sensor reading.
        # This creates the error signal fed into the controller.
        feedback_signal = self.robot.position + self.light_perturbation
        error = 0 - feedback_signal

        # 2. PID controller calculates the required motor voltage.
        pid_output, p, i, d = self.pid.update(error)

        # 3. The motor acts on the robot.
        self.robot.update(pid_output)

        # 4. Movement perturbation (Pf(t)/Pi(t)) acts as a direct physical push on the robot.
        self.robot.position += self.movement_perturbation
        response = self.robot.position # The final response includes this push

        # 5. Log data for plotting.
        total_pert = self.light_perturbation + self.movement_perturbation
        self.log_data(error, p, i, d, response, total_pert)
        self.time += self.dt

        # 6. Reset perturbations (they are Dirac-like, lasting one step).
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

        return self.history

    def log_data(self, error, p, i, d, response, total_pert):
        data_to_log = [self.time, error, p, i, d, response, total_pert]
        for key, val in zip(self.history.keys(), data_to_log):
            self.history[key].append(val)
            if len(self.history[key]) > POINTS_OF_HISTORY:
                self.history[key].pop(0)

    def inject_light_perturbation(self, amplitude):
        self.light_perturbation = amplitude

    def inject_movement_perturbation(self, amplitude):
        self.movement_perturbation = amplitude

# --- GUI Application ---

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("PID Line Follower Simulation")
        self.geometry("1200x800")

        self.simulation = Simulation(dt=0.05)

        # Configure grid layout
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create frames
        self.controls_frame = ctk.CTkFrame(self, width=300)
        self.controls_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ns")
        self.controls_frame.grid_propagate(False)

        self.plot_frame = ctk.CTkFrame(self)
        self.plot_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.setup_controls()
        self.setup_plots()

        self.running = True
        self.after(50, self.update_loop)

    def setup_controls(self):
        self.kp_slider = self.create_slider("Kp", 0, 10, 1.0)
        self.ki_slider = self.create_slider("Ki", 0, 2, 0.1)
        self.kd_slider = self.create_slider("Kd", 0, 1, 0.01)

        light_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Light Perturbation", command=self.inject_light)
        light_pert_button.pack(pady=15, padx=10, fill='x')

        move_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Movement Perturbation", command=self.inject_movement)
        move_pert_button.pack(pady=15, padx=10, fill='x')

    def create_slider(self, text, from_, to, initial_value):
        frame = ctk.CTkFrame(self.controls_frame)
        label = ctk.CTkLabel(frame, text=f"{text}: {initial_value:.2f}", width=100)
        label.pack(side='left', padx=10)
        
        def slider_command(value):
            label.configure(text=f"{text}: {value:.2f}")
            self.update_pid_gains()

        slider = ctk.CTkSlider(frame, from_=from_, to=to, command=slider_command)
        slider.set(initial_value)
        slider.pack(side='left', expand=True, fill='x', padx=10)
        frame.pack(pady=10, padx=10, fill="x")
        return slider

    def update_pid_gains(self):
        kp = self.kp_slider.get()
        ki = self.ki_slider.get()
        kd = self.kd_slider.get()
        self.simulation.pid.set_gains(kp, ki, kd)

    def inject_light(self):
        self.simulation.inject_light_perturbation(amplitude=5.0)

    def inject_movement(self):
        self.simulation.inject_movement_perturbation(amplitude=-5.0)

    def setup_plots(self):
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        self.fig.subplots_adjust(hspace=0.8, bottom=0.1, top=0.95)
        plt.style.use('seaborn-v0_8-darkgrid')

        self.lines = {
            'pert': self.axes[0].plot([], [], label='Total Perturbations')[0],
            'error': self.axes[1].plot([], [], label='Error Signal (e)')[0],
            'pid_p': self.axes[2].plot([], [], label='P')[0],
            'pid_i': self.axes[2].plot([], [], label='I')[0],
            'pid_d': self.axes[2].plot([], [], label='D')[0],
            'resp': self.axes[3].plot([], [], label='Response (θₒ)')[0]
        }

        self.axes[0].set_title("Perturbations")
        self.axes[1].set_title("Error Signal (e)")
        self.axes[2].set_title("PID Controller Outputs")
        self.axes[3].set_title("System Response (Position)")
        self.axes[3].set_xlabel("Time (s)")

        self.axes[0].set_ylim(-10, 10)
        self.axes[1].set_ylim(-10, 10)
        self.axes[2].set_ylim(-20, 20)
        self.axes[3].set_ylim(-10, 10)

        for i, ax in enumerate(self.axes):
            if i == 2:
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.45), ncol=3)
            else:
                ax.legend()
            ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(side=ctk.TOP, fill=ctk.BOTH, expand=True)

    def update_plots(self, history):
        time_data = history['time']
        self.lines['pert'].set_data(time_data, history['total_pert'])
        self.lines['error'].set_data(time_data, history['error'])
        self.lines['pid_p'].set_data(time_data, history['p_out'])
        self.lines['pid_i'].set_data(time_data, history['i_out'])
        self.lines['pid_d'].set_data(time_data, history['d_out'])
        self.lines['resp'].set_data(time_data, history['response'])

        if time_data:
            min_time, max_time = time_data[0], time_data[-1]
            if min_time == max_time:
                max_time = min_time + self.simulation.dt
            for ax in self.axes:
                ax.set_xlim(min_time, max_time)
        self.canvas.draw()

    def update_loop(self):
        if self.running:
            history = self.simulation.update()
            self.update_plots(history)
            self.after(50, self.update_loop)

    def on_closing(self):
        self.running = False
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()