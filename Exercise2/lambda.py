import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Plotter:
    def __init__(self, t_min=0, t_max=10, dt=0.01):
        self.t_min = t_min
        self.t_max = t_max
        self.dt = dt
        self.t = np.arange(t_min, t_max, dt)
        
        # Compute initial data for h(t)
        self.h_values = self.h(self.t)
        
    def lambda_func(self, t):
        """Lambda function: λ(t) = 5 * sin(2 * pi * t)"""
        return 5 * np.sin(2 * np.pi * t)

    def h(self, t):
        """Main function: h(t) = 3 * pi * exp(-lambda(t))"""
        return 3 * np.pi * np.exp(-self.lambda_func(t))
    
    def plot_static(self):
        """Static plot of h(t) over the given time range"""
        plt.figure(figsize=(10, 6))
        plt.plot(self.t, self.h_values, label="h(t)")
        plt.xlabel("Time (t)")
        plt.ylabel("h(t)")
        plt.title("Plot of h(t) = 3π * exp(-λ(t))")
        plt.legend()
        plt.grid(True)
        plt.show()
        
    def update_data(self, t_new):
        """Update the h(t) values for dynamic plotting"""
        self.t = np.append(self.t, t_new)
        new_h_value = self.h(t_new)
        self.h_values = np.append(self.h_values, new_h_value)

class PlotterLive(Plotter):
    def __init__(self, t_min=0, t_max=10, dt=0.01):
        super().__init__(t_min, t_max, dt)
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, 
                                 frames=np.arange(t_min, t_max, dt), interval=50, blit=True)
    
    def init_plot(self):
        """Initialize the plot for animation"""
        self.ax.set_xlim(self.t_min, self.t_max)
        self.ax.set_ylim(np.min(self.h_values), np.max(self.h_values))
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, t_new):
        """Update the plot dynamically with new data"""
        self.update_data(t_new)
        self.line.set_data(self.t, self.h_values)
        self.ax.set_xlim(self.t.min(), self.t.max())
        self.ax.set_ylim(np.min(self.h_values), np.max(self.h_values))
        return self.line,

    def show(self):
        plt.show()

from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton, QSlider, QLabel, QLineEdit, QHBoxLayout
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import sys

class PlotterGUI(QWidget):
    def __init__(self, plotter):
        super().__init__()
        self.plotter = plotter  # Instance of PlotterLive class

        self.initUI()

    def initUI(self):
        self.setWindowTitle("Real-time Plotter GUI")

        # Create main layout
        layout = QVBoxLayout()

        # Create matplotlib canvas for embedding in PyQt
        self.canvas = FigureCanvas(self.plotter.fig)
        layout.addWidget(self.canvas)

        # Create Start, Stop, Reset buttons
        button_layout = QHBoxLayout()

        start_btn = QPushButton('Start', self)
        start_btn.clicked.connect(self.start_plotting)
        button_layout.addWidget(start_btn)

        stop_btn = QPushButton('Stop', self)
        stop_btn.clicked.connect(self.stop_plotting)
        button_layout.addWidget(stop_btn)

        reset_btn = QPushButton('Reset', self)
        reset_btn.clicked.connect(self.reset_plotting)
        button_layout.addWidget(reset_btn)

        layout.addLayout(button_layout)

        # Add a zoom slider
        zoom_layout = QHBoxLayout()
        zoom_label = QLabel('Zoom:', self)
        zoom_slider = QSlider(Qt.Horizontal)
        zoom_slider.setMinimum(1)
        zoom_slider.setMaximum(100)
        zoom_slider.setValue(10)
        zoom_slider.setTickPosition(QSlider.TicksBelow)
        zoom_slider.setTickInterval(5)
        zoom_slider.valueChanged.connect(self.zoom_plot)
        zoom_layout.addWidget(zoom_label)
        zoom_layout.addWidget(zoom_slider)

        layout.addLayout(zoom_layout)

        # Add an edit field to name the experiment
        experiment_layout = QHBoxLayout()
        experiment_label = QLabel('Experiment Name:', self)
        self.experiment_field = QLineEdit(self)
        experiment_layout.addWidget(experiment_label)
        experiment_layout.addWidget(self.experiment_field)
        layout.addLayout(experiment_layout)

        # Add a save button
        save_btn = QPushButton('Save Data', self)
        save_btn.clicked.connect(self.save_data)
        layout.addWidget(save_btn)

        self.setLayout(layout)

    def start_plotting(self):
        """Start real-time plotting"""
        self.plotter.ani.event_source.start()

    def stop_plotting(self):
        """Stop the real-time plotting"""
        self.plotter.ani.event_source.stop()

    def reset_plotting(self):
        """Reset the plot data and reinitialize"""
        self.plotter.t = np.arange(self.plotter.t_min, self.plotter.t_max, self.plotter.dt)
        self.plotter.h_values = self.plotter.h(self.plotter.t)
        self.plotter.line.set_data(self.plotter.t, self.plotter.h_values)
        self.plotter.ax.set_xlim(self.plotter.t_min, self.plotter.t_max)
        self.plotter.ax.set_ylim(np.min(self.plotter.h_values), np.max(self.plotter.h_values))
        self.canvas.draw()

    def zoom_plot(self, value):
        """Zoom in/out by adjusting x and y limits"""
        scale_factor = value / 10.0
        self.plotter.ax.set_xlim(self.plotter.t_min, self.plotter.t_max / scale_factor)
        self.plotter.ax.set_ylim(np.min(self.plotter.h_values), np.max(self.plotter.h_values) / scale_factor)
        self.canvas.draw()

    def save_data(self):
        """Save the current data to a CSV file"""
        experiment_name = self.experiment_field.text()
        if not experiment_name:
            experiment_name = "experiment"

        filename = f"{experiment_name}_{self.get_current_time()}.csv"
        data = np.column_stack((self.plotter.t, self.plotter.h_values))
        np.savetxt(filename, data, delimiter=',', header='Time,h(t)', comments='')

        print(f"Data saved as {filename}")

    def get_current_time(self):
        """Get current time for file naming"""
        from datetime import datetime
        return datetime.now().strftime("%Y%m%d_%H%M%S")

# Main function to run the GUI
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create plotter instance
    plotter = PlotterLive(t_min=0, t_max=10, dt=0.1)

    # Create GUI instance
    gui = PlotterGUI(plotter)
    gui.show()

    sys.exit(app.exec_())
