import customtkinter as ctk
import tkinter.messagebox as messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import control
import utils

# --- Constants ---
## Interface
SCAN_TIME = 50                  # (ms)      # Controls how often the interface updates on screen
VISIBLE_SECONDS = 5

## Sliders
INITIAL_REFERENCE_VALUE = 0.0   # (px)
INITIAL_KP = 1.5
INITIAL_KI = 0.1
INITIAL_KD = 0.6

class SliceConfig:
    def __init__(self, name, min, max, initial):
        self.name = name
        self.min = min
        self.max = max
        self.initial = initial
        self.label = None

KP_CONFIG = SliceConfig(name="Kp", min=0.0, max=10.0, initial=INITIAL_KP)
KI_CONFIG = SliceConfig(name="Ki", min=0.0, max=0.9, initial=INITIAL_KI)
KD_CONFIG = SliceConfig(name="Kd", min=0.0, max=1.0, initial=INITIAL_KD)
REFERENCE_CONFIG = SliceConfig(name="Reference", min=-5.0, max=5.0, initial=INITIAL_REFERENCE_VALUE)

# --- GUI Application ---

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("PID Line Follower Simulation")
        self.attributes('-fullscreen', True)
        self.simulation = control.Simulation(KP_CONFIG.initial, KD_CONFIG.initial, KI_CONFIG.initial, REFERENCE_CONFIG.initial)
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
        self.plot_frame.grid_propagate(False)
        self.plot_frame.grid_rowconfigure(0, weight=1)
        self.plot_frame.grid_columnconfigure(0, weight=1)

        self.setup_controls()
        self.setup_plots()
        self.update_visible_graphs()

        self.running = True
        self.paused = False
        self.after_id = self.after(SCAN_TIME, self.update_loop)

    def setup_controls(self):
        self.kp_slider = self.create_slider(KP_CONFIG)
        self.ki_slider = self.create_slider(KI_CONFIG)
        self.kd_slider = self.create_slider(KD_CONFIG)
        self.reference_slider = self.create_slider(REFERENCE_CONFIG, self.update_reference_value)

        light_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Light Perturbation", command=self.inject_light)
        light_pert_button.pack(pady=15, padx=10, fill='x')

        move_pert_button = ctk.CTkButton(self.controls_frame, text="Inject Movement Perturbation", command=self.inject_movement)
        move_pert_button.pack(pady=15, padx=10, fill='x')

        self.graph_toggles = []
        graph_names = [
            "Perturbations", "Error", "PID", "Reference",
            "Feedback", "Position & Velocity"
        ]

        for i, name in enumerate(graph_names):
            toggle = ctk.CTkCheckBox(
                self.controls_frame,
                text=f"Show {name}",
                command=self.update_visible_graphs
            )
            toggle.select()
            toggle.pack(padx=10, pady=3, anchor="w")
            self.graph_toggles.append(toggle)

        separator = ctk.CTkLabel(self.controls_frame, text="")
        separator.pack(pady=10)

        buttons_frame = ctk.CTkFrame(self.controls_frame)
        buttons_frame.pack(side="bottom", pady=10, padx=10, fill='x')

        self.pause_button = ctk.CTkButton(buttons_frame, text="Pause", command=self.toggle_pause)
        self.pause_button.pack(side="left", pady=10, padx=10, fill='x', expand=True)

        exit_button = ctk.CTkButton(buttons_frame, text="Close", command=self.on_closing)
        exit_button.pack(side="left", pady=10, padx=10, fill='x', expand=True)
        
        reset_button = ctk.CTkButton(self.controls_frame, text="Reset", command=self.reset_simulation)
        reset_button.pack(side="bottom", pady=10, padx=10, fill='x')

    def update_visible_graphs(self):
        height_ratios_all = [1, 1, 2, 1, 1, 1.5]
        spacing = 0.05
        top = 0.95
        bottom = 0.1
        visible_flags = [chk.get() == 1 for chk in self.graph_toggles]
        visible_indices = [i for i, v in enumerate(visible_flags) if v]
        visible_ratios = [height_ratios_all[i] for i in visible_indices]

        if not visible_indices:
            self.canvas.draw()
            return

        total_ratio = sum(visible_ratios)
        total_spacing = spacing * (len(visible_indices) - 1)
        available_height = top - bottom - total_spacing

        current_top = top

        for i in range(len(self.axes)):
            if visible_flags[i]:
                ratio = height_ratios_all[i] / total_ratio
                height = available_height * ratio
                bottom_pos = current_top - height
                self.axes[i].set_position([0.1, bottom_pos, 0.85, height])
                self.axes[i].set_visible(True)
                current_top = bottom_pos - spacing
            else:
                self.axes[i].set_position([0, 0, 0, 0])
                self.axes[i].set_visible(False)

        self.canvas.draw()

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
        self.simulation.pid.reset_gains(kp, kd, ki)

    def inject_light(self):
        self.simulation.inject_light_perturbation(amplitude=utils.generate_signed_random(3.0, 5.0))

    def inject_movement(self):
        self.simulation.inject_movement_perturbation(amplitude=utils.generate_signed_random(5.0, 8.0))

    def setup_plots(self):
        self.fig, self.axes = plt.subplots(6, 1, figsize=(10, 10), sharex=True, gridspec_kw={'height_ratios': [1, 1, 2, 1, 1, 1.5]})
        self.fig.subplots_adjust(hspace=0.8, bottom=0.1, top=0.95)
        plt.style.use('seaborn-v0_8-darkgrid')

        self.lines = {
            'pert': self.axes[0].plot([], [], color='tab:blue')[0],
            'error': self.axes[1].plot([], [], color='tab:blue')[0],
            'pid_p': self.axes[2].plot([], [], label='P', color='tab:blue')[0],
            'pid_i': self.axes[2].plot([], [], label='I', color='tab:orange')[0],
            'pid_d': self.axes[2].plot([], [], label='D', color='tab:green')[0],
            'reference': self.axes[3].plot([], [], color='tab:blue')[0],
            'feedback': self.axes[4].plot([], [], color='tab:blue')[0],
            'velocity': self.axes[5].plot([], [], label='Velocity', linestyle='--', color='tab:orange')[0],
            'resp': self.axes[5].plot([], [], label='Position (θₒ)', color='tab:blue')[0]
        }

        self.axes[0].set_title("Perturbations", loc='left', fontsize=12)
        self.axes[1].set_title("Error Signal (e)", loc='left', fontsize=12)
        self.axes[2].set_title("PID Controller Outputs", loc='left', fontsize=12)
        self.axes[3].set_title("Reference Value", loc='left', fontsize=12)
        self.axes[4].set_title("Feedback Signal", loc='left', fontsize=12)
        self.axes[5].set_title("System Response", loc='left', fontsize=12)
        self.axes[5].set_xlabel("Time (s)")

        self.axes[0].set_ylim(-10, 10)
        self.axes[1].set_ylim(-10, 10)
        self.axes[2].set_ylim(-40, 40)
        self.axes[3].set_ylim(-6, 6)
        self.axes[4].set_ylim(-10, 10)
        self.axes[5].set_ylim(-10, 10)
        self.axes[5].axhspan(-(control.LINE_WIDTH)/2, control.LINE_WIDTH/2, facecolor='lightgreen', alpha=0.7, label='Line limits')

        self.axes[0].set_yticks(np.arange(-10, 11, 5))
        self.axes[1].set_yticks(np.arange(-10, 11, 5))
        self.axes[2].set_yticks(np.arange(-40, 41, 10))
        self.axes[3].set_yticks(np.arange(-6, 7, 5))
        self.axes[4].set_yticks(np.arange(-10, 11, 5))
        self.axes[5].set_yticks(np.arange(-10, 11, 5))

        for i, ax in enumerate(self.axes):
            if i == 2:
                ax.legend(loc='upper right', bbox_to_anchor=(1, 1), ncol=3)
            else:
                _, labels = ax.get_legend_handles_labels()
                if labels:
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
        self.lines['velocity'].set_data(time_data, history['velocity'])
        self.lines['resp'].set_data(time_data, history['response'])
        self.lines['reference'].set_data(time_data, history['reference'])
        self.lines['feedback'].set_data(time_data, history['feedback_signal'])

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

    def save_full_plot(self):
        history = self.simulation.history.data
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

        self.fig.savefig("assets/simulacion_pid.png", dpi=150)

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()