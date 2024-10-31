# :octocat: Robot Controller

**Robot Controller** is a Python script designed to manage an iRobot Create2 (Roomba) through manual and automatic modes. The script leverages the `pycreate2` library for direct communication with the robot, with real-time data visualization using `matplotlib` and `tkinter` for a GUI.

## Functionality

### 1. **Modes of Operation:**
   - **Manual Mode**: Enables direct control of the robot using keyboard arrows, providing instant movement adjustments and testing flexibility.
   - **Automatic Mode**: Runs the robot on a pre-defined path, ideal for testing route-following capabilities and movement precision.

### 2. **Odometry and Speed Control:**
   - **Position Calculation**: The script continuously updates the robot's position `(x, y)` and orientation using encoder data, calculated through `Odometer`.
   - **Speed Control**: Both modes allow adjustable speed; in automatic mode, the speed is set to follow the programmed path autonomously.

### 3. **Graphical Visualization (GUI):**
   - **Interface**: The GUI displays the robot’s current position and orientation, facilitating real-time trajectory tracking.
   - **Trajectory Update**: The interface uses `matplotlib` to plot the ongoing path and `tkinter` for interactive control windows. In manual mode, users can watch the position change with each movement command.

## Main Classes

### `RobotController`

This class handles the connection and communication with the robot, managing both manual and automatic movement. It is also responsible for odometry updates, calculating and visualizing the robot's real-time position.

   - **Attributes**:
     - `self.port`, `self.baud`: Communication port and baud rate for robot connection.
     - `self.bot`: Instance of `Create2` from `pycreate2`, enabling robot interaction.
     - `self.odometer`: An instance of `Odometer` to calculate position based on encoder data.
     - `self.queue`: Queue structure to manage movement commands and data.
     - `self.fig`, `self.ax`: Elements from `matplotlib` for GUI plotting.

   - **Primary Methods**:
     - `initialize_robot()`: Establishes the connection with the robot and sets up the initial GUI configurations, loading the iRobot image and creating the coordinate grid.
     - `manual_control()`: Allows direct movement commands through keyboard inputs.
     - `automatic_control()`: Runs a pre-set series of movements autonomously.
     - `draw_robot()`: Updates the robot's position in the GUI, showing real-time trajectory and orientation.

### `Odometer`

The `Odometer` class handles odometry calculations, enabling precise tracking of position and orientation based on encoder readings.

   - **Attributes**:
     - `self.bot`: Reference to the `Create2` instance.
     - `self.x`, `self.y`: Current coordinates of the robot.
     - `self.angle`: Orientation in radians.

   - **Primary Methods**:
     - `initialize_odometry()`: Sets initial values for position and orientation.
     - `update_odometry()`: Updates `(x, y)` position and `angle` based on encoder readings, calculating distance and rotation traveled.

## Running the Script

1. **Initial Configuration**: Connect the iRobot to the computer’s serial port (default: `COM3`). Modify the port if needed.

2. **Run the Script**: Open a terminal and execute:
   ```bash
   python go.py
