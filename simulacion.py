import random
import customtkinter as ctk
import tkinter.messagebox as messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --- Constants ---
class SliceConfig:
    def __init__(self, name, min, max, initial):
        self.name = name
        self.min = min
        self.max = max
        self.initial = initial
        self.label = None

KP_CONFIG = SliceConfig(name="Kp", min=0.0, max=10.0, initial=1.5)
KI_CONFIG = SliceConfig(name="Ki", min=0.0, max=0.9, initial=0.1)
KD_CONFIG = SliceConfig(name="Kd", min=0.0, max=1.0, initial=0.6)
REFERENCE_CONFIG = SliceConfig(name="Reference", min=-5.0, max=5.0, initial=0.0)

## Robot
ROBOT_MASS = 1.0

## Context
FLOOR_DAMPING = 0.5
LINE_WIDTH = 1.8

## Simulation
SCAN_TIME = 50
PI_LOOP_TIME = 10
PI_LOOP_ITERATIONS = 4
POINTS_OF_HISTORY = 500  # Number of data points to show on the graphs
VISIBLE_SECONDS = 5

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
        # Proportional
        if abs(error) > 1:
            p_term = self.Kp * error
        else:    
            p_term = 0

        # Integral
        self.integral += error * self.dt
        i_term = self.Ki * self.integral

        # Derivative
        if abs(error - self.previous_error) > 3 :
            derivative = (error - self.previous_error) / self.dt
        else: 
            derivative = 0
        d_term = self.Kd * derivative
        self.previous_error = error

        return p_term + i_term + d_term, p_term, i_term, d_term

    def reset_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # Reset state when gains change to avoid instability
        self.integral = 0.0
        self.previous_error = 0.0

class RobotModel:
    """Simulates the robot's physical dynamics (the 'plant')."""
    def __init__(self, dt):
        self.dt = dt
        self.x_axis_position = 0.0 # En el centro de la línea
        self.x_axis_velocity = 0.0 # Sigue derecho
        self.velocity = 10  # Initial velocity

    def update(self, motor_voltage_change):
        # Positivo indica que el motor empuja hacia la derecha
        # Negativo indica que el motor empuja hacia la izquierda
        x_axis_acceleration = (motor_voltage_change - FLOOR_DAMPING * self.x_axis_velocity) / ROBOT_MASS
        self.x_axis_velocity += x_axis_acceleration * self.dt
        self.x_axis_position += self.x_axis_velocity * self.dt
        return self.x_axis_position

class Simulation:
    """Manages the simulation based on the block diagram."""
    def __init__(self, dt=0.1):
        self.dt = dt
        self.time = 0.0
        self.pid = PIDController(KP_CONFIG.initial, KI_CONFIG.initial, KD_CONFIG.initial, self.dt)
        self.robot = RobotModel(self.dt)
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0
        self.reference = REFERENCE_CONFIG.initial
        self.history = {k: [] for k in ['time', 'error', 'p_out', 'i_out', 'd_out', 'response', 'total_pert', 'velocity', 'reference']}

    def update(self):
        # 1. Light perturbation (Pl(t)) affects the sensor reading.
        # This creates the error signal fed into the controller.
        feedback_signal = self.robot.x_axis_position + self.light_perturbation
        error = self.reference - feedback_signal

        # 2. PID controller calculates the required motor voltage.
        pid_output, p, i, d = self.pid.update(error)

        # 3. The motor acts on the robot.
        self.robot.update(pid_output)

        # 4. Movement perturbation (Pf(t)/Pi(t)) acts as a direct physical push on the robot.
        self.robot.x_axis_position += self.movement_perturbation
        response = self.robot.x_axis_position # The final response includes this push

        # 5. Log data for plotting.
        total_pert = self.light_perturbation + self.movement_perturbation
        self.log_data(error, p, i, d, response, total_pert, self.reference)
        self.time += self.dt

        # 6. Reset perturbations (they are Dirac-like, lasting one step).
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

        return self.history

    def log_data(self, error, p, i, d, response, total_pert, reference):
        data_to_log = [self.time, error, p, i, d, response, total_pert, self.robot.x_axis_velocity, reference]
        for key, val in zip(self.history.keys(), data_to_log):
            self.history[key].append(val)
            if len(self.history[key]) > POINTS_OF_HISTORY:
                self.history[key].pop(0)

    def inject_light_perturbation(self, amplitude):
        self.light_perturbation = amplitude

    def inject_movement_perturbation(self, amplitude):
        self.movement_perturbation = amplitude

    def reset(self):
        self.time = 0.0
        self.robot = RobotModel(self.dt)
        self.light_perturbation = 0.0
        self.movement_perturbation = 0.0

        # Limpiar historial de gráficos
        for key in self.history:
            self.history[key].clear()

# --- GUI Application ---

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("PID Line Follower Simulation")
        self.attributes('-fullscreen', True)

        self.simulation = Simulation(dt=0.1)
        self.reference_slider = None

        # Configure grid layout
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create frames
        self.controls_frame = ctk.CTkFrame(self)
        self.controls_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ns")
        self.controls_frame.grid_rowconfigure(0, weight=1)
        self.controls_frame.grid_columnconfigure(0, weight=1)

        self.plot_frame = ctk.CTkFrame(self)
        self.plot_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.setup_controls()
        self.setup_plots()

        self.running = True
        self.paused = False
        self.after_id = self.after(50, self.update_loop)

    def setup_controls(self):
        self.kp_slider = self.create_slider(KP_CONFIG)
        self.ki_slider = self.create_slider(KI_CONFIG)
        self.kd_slider = self.create_slider(KD_CONFIG)
        self.reference_slider = self.create_slider(REFERENCE_CONFIG, self.update_reference_value)

        light_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Light Perturbation", command=self.inject_light)
        light_pert_button.pack(pady=15, padx=10, fill='x')

        move_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Movement Perturbation", command=self.inject_movement)
        move_pert_button.pack(pady=15, padx=10, fill='x')

        buttons_frame = ctk.CTkFrame(self.controls_frame)
        buttons_frame.pack(side="bottom", pady=10, padx=10, fill='x')

        self.pause_button = ctk.CTkButton(buttons_frame, text="Pause", command=self.toggle_pause)
        self.pause_button.pack(side="left", pady=10, padx=10, fill='x', expand=True)

        exit_button = ctk.CTkButton(buttons_frame, text="Close", command=self.on_closing)
        exit_button.pack(side="left", pady=10, padx=10, fill='x', expand=True)
        
        reset_button = ctk.CTkButton(self.controls_frame, text="Reset", command=self.reset_simulation)
        reset_button.pack(side="bottom", pady=10, padx=10, fill='x')

    def create_slider(self, controller_config, command=None):
        text = controller_config.name
        initial_value = controller_config.initial

        frame = ctk.CTkFrame(self.controls_frame)
        label = ctk.CTkLabel(frame, text=f"{text}: {initial_value:.2f}", width=100)
        label.pack(side='left', padx=10)
        
        def slider_command(value):
            label.configure(text=f"{text}: {value:.2f}")
            if command:
                command(value)
            else:
                self.update_pid_gains()

        controller_config.label = label

        slider = ctk.CTkSlider(frame, from_=controller_config.min, to=controller_config.max, command=slider_command)
        slider.set(initial_value)
        slider.pack(side='left', expand=True, fill='x', padx=10)
        frame.pack(pady=10, padx=10, fill="x")
        return slider

    def update_reference_value(self, value):
        self.simulation.reference = value

    def update_pid_gains(self):
        kp = self.kp_slider.get()
        ki = self.ki_slider.get()
        kd = self.kd_slider.get()
        self.simulation.pid.reset_gains(kp, ki, kd)

    def inject_light(self):
        self.simulation.inject_light_perturbation(amplitude=self.generate_signed_random(3.0, 5.0))

    def inject_movement(self):
        self.simulation.inject_movement_perturbation(amplitude=self.generate_signed_random(5.0, 8.0))

    def setup_plots(self):
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True, gridspec_kw={'height_ratios': [1, 1, 2, 1.5]})
        self.fig.subplots_adjust(hspace=0.8, bottom=0.1, top=0.95)
        plt.style.use('seaborn-v0_8-darkgrid')

        self.lines = {
            'pert': self.axes[0].plot([], [], label='Total Perturbations')[0],
            'error': self.axes[1].plot([], [], label='Error Signal (e)')[0],
            'pid_p': self.axes[2].plot([], [], label='P')[0],
            'pid_i': self.axes[2].plot([], [], label='I')[0],
            'pid_d': self.axes[2].plot([], [], label='D')[0],
            'resp': self.axes[3].plot([], [], label='Response (θₒ)')[0],
            'velocity': self.axes[3].plot([], [], label='Velocity', linestyle='--')[0],
            'reference': self.axes[3].plot([], [], label='Reference', linestyle=':')[0]
        }

        self.axes[0].set_title("Perturbations")
        self.axes[1].set_title("Error Signal (e)")
        self.axes[2].set_title("PID Controller Outputs")
        self.axes[3].set_title("System Response (Position)")
        self.axes[3].set_xlabel("Time (s)")

        self.axes[0].set_ylim(-10, 10)
        self.axes[1].set_ylim(-10, 10)
        self.axes[2].set_ylim(-30, 30)
        self.axes[3].set_ylim(-10, 10)
        self.axes[3].axhspan(-LINE_WIDTH/2, LINE_WIDTH/2, facecolor='lightgreen', alpha=0.7, label='Line limits')

        self.axes[0].set_yticks(np.arange(-10, 11, 5))
        self.axes[1].set_yticks(np.arange(-10, 11, 5))
        self.axes[2].set_yticks(np.arange(-30, 31, 10))
        self.axes[3].set_yticks(np.arange(-10, 11, 5))

        for i, ax in enumerate(self.axes):
            if i == 2:
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3)
            else:
                ax.legend(loc='upper right')
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
        self.lines['velocity'].set_data(time_data, history['velocity'])
        self.lines['reference'].set_data(time_data, history['reference'])

        if time_data:
            max_time = time_data[-1]
            min_time = max_time - VISIBLE_SECONDS
            for ax in self.axes:
                ax.set_xlim(min_time, max_time)
        self.canvas.draw()

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.pause_button.configure(text="Resume")
        else:
            self.pause_button.configure(text="Pause")

    def update_loop(self):
        if self.running and not self.paused:
            history = self.simulation.update()
            self.update_plots(history)

        if self.running:
            self.after_id = self.after(SCAN_TIME, self.update_loop)

    def on_closing(self):
        self.running = False

        save_plot = messagebox.askyesno(
            "Save Graph",
            "Do you want to save an image of the full graph?"
        )
        if save_plot:
            self.save_full_plot()

        after_id = getattr(self, 'after_id', None)
        if after_id:
            try:
                self.after_cancel(after_id)
            except Exception:
                pass

        try:
            app.withdraw()
            app.quit()
        except Exception:
            pass

    def reset_simulation(self):
        self.kp_slider.set(KP_CONFIG.initial)
        KP_CONFIG.label.configure(text=f"{KP_CONFIG.name}: {KP_CONFIG.initial:.2f}")
        self.ki_slider.set(KI_CONFIG.initial)
        KI_CONFIG.label.configure(text=f"{KI_CONFIG.name}: {KI_CONFIG.initial:.2f}")
        self.kd_slider.set(KD_CONFIG.initial)
        KD_CONFIG.label.configure(text=f"{KD_CONFIG.name}: {KD_CONFIG.initial:.2f}")
        self.reference_slider.set(REFERENCE_CONFIG.initial)
        REFERENCE_CONFIG.label.configure(text=f"{REFERENCE_CONFIG.name}: {REFERENCE_CONFIG.initial:.2f}")

        self.simulation.reset()
        self.update_pid_gains()
        self.update_reference_value(REFERENCE_CONFIG.initial)

    def generate_signed_random(self, min_value, max_value):
        rnd = random.uniform(-max_value, max_value)
        if (abs(rnd) < min_value):
            return self.generate_signed_random(min_value, max_value)
        return rnd

    def save_full_plot(self):
        history = self.simulation.history
        time_data = history['time']
        if not time_data:
            return

        self.lines['pert'].set_data(time_data, history['total_pert'])
        self.lines['error'].set_data(time_data, history['error'])
        self.lines['pid_p'].set_data(time_data, history['p_out'])
        self.lines['pid_i'].set_data(time_data, history['i_out'])
        self.lines['pid_d'].set_data(time_data, history['d_out'])
        self.lines['resp'].set_data(time_data, history['response'])
        self.lines['velocity'].set_data(time_data, history['velocity'])
        self.lines['reference'].set_data(time_data, history['reference'])

        min_time = time_data[0]
        max_time = time_data[-1]
        for ax in self.axes:
            ax.set_xlim(min_time, max_time)

        self.fig.savefig("simulacion_pid.png", dpi=150)

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()