# Install Guide for CI-PX4

# PBAR-Enhanced PX4 Drone Autopilot

This repository holds the PBAR-enhanced [PX4](http://px4.io/) flight control solution for drones, with the main applications located in the src/modules directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

The PBAR enhancement modifies codes in the following locations:

- Core: `src/modules/sensors/software_sensor`
- Sensor Attack & Anomaly Detection
    - `sensor_attack` library: `src/lib/sensor_attack`
    - `fault_detector` library: `src/lib/fault_detector`
    - GPS: `src/modules/sensors/vehicle_gps_position`
    - Barometer: `src/modules/sensors/vehicle_air_data`
    - Magnetometer
        - `src/modules/ekf2` (Detection)
        - `src/modules/sensors/vehicle_magnetometer` (Attack)
    - Accelerometer: `src/lib/drivers/accelerometer`
    - Gyroscope: `src/lib/drivers/gyroscope`
- Recovery: `src/modules/ekf2`

## Getting Started

PX4 firmware can be built from source code on the console or in an IDE, for both simulated and hardware targets. You need to build PX4 in order to use simulators, or if you want to modify PX4 and create a custom build.

Before building the PX4, you must first install the **Developer Toolchain** for your host operating system and target hardware.

### Install Developer Toolchain

The following instructions use a bash script set up the PX4 development environment on Ubuntu Linux 18.04 amd 20.04.

**NOTE:** The script is intended to be run on *clean* Ubuntu LTS installations, and may not work if run "on top" of an existing system, or on a different Ubuntu release.

The environment includes:

- Gazebo Classic Simulator on Ubuntu 20.04 and Ubuntu 18.04
- Build toolchain for Pixhawk (and other NuttX-based hardware).

This instruction is intended for **simulation and NuttX (Pixhawk) targets**. For other target platform please refer to [https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html](https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html).

To install the toolchain:

1. Choose an option to obtain **ubuntu.sh** and **requirements.txt**:
    1. **Recommend**: Download PX4 source code from the repo. The environment setup scripts in the source usually work for recent releases.

       Clone any branch in `baseline` (e.g., `baseline/CI` ) to folder `PX4-Autopilot`:

        ```bash
        git clone -b baseline/CI --recursive https://github.com/wangwwno1/Project-VIMU.git PX4-Autopilot
        ```

       Run the **ubuntu.sh** with no arguments (in a bash shell) to install everything:

        ```bash
        bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
        ```

    2. Alternatively, if you just wanted to set up the development environment without getting all the source code, just download **ubuntu.sh** and **requirements.txt** and then run **ubuntu.sh**:

        ```bash
        wget https://raw.githubusercontent.com/wangwwno1/Project-VIMU/baseline/CI/Tools/setup/ubuntu.sh
        wget https://raw.githubusercontent.com/wangwwno1/Project-VIMU/baseline/CI/Tools/setup/requirements.txt
        bash ubuntu.sh
        ```

2. Restart the computer on completion.
3. **IMPORTANT** Run the following command lines to install the specific version of empy.

   PX4 1.13.3 relies on attributes that are removed in empy 4.x.

   Rollback the package version with following commands:

    ```bash
    pip uninstall empy
    pip install empy==3.3.4
    ```


### Building PX4 Software

The enhanced PX4 source code is stored on Github in the [wangwwno1/Project-VIMU](https://github.com/wangwwno1/Project-VIMU) repository. To get the VIMU (`baseline/CI` branch) enhanced PX4 onto your computer, enter the following command into a terminal:

```bash
git clone -b baseline/CI --recursive https://github.com/wangwwno1/Project-VIMU.git PX4-Autopilot
```

Note that you may already have done this when installing the **Developer Toolchain.** If needed you can also [**get the source code specific to a particular release**](https://docs.px4.io/main/en/contribute/git_examples.html#get-a-specific-release). [**GIT Examples**](https://docs.px4.io/main/en/contribute/git_examples.html) provides a lot more information working with releases and contributing to PX4.

### First Build (Using a Simulator)

First we'll build a simulated target using a console environment. This allows us to validate the system setup before moving on to real hardware and an IDE.

Navigate into the **PX4-Autopilot** directory and start [**jMAVSim**](https://docs.px4.io/v1.13/en/simulation/jmavsim.html) using the following command:

```bash
make px4_sitl jmavsim
```

This will bring up the PX4 console. The first time build may invoke numerous warning message. Set parameter `IMU_GYRO_RATEMAX` to 250 and restart the autopilot will suppress them:

```bash
pxh> param set IMU_GYRO_RATEMAX 250
```

You may need to start *QGroundControl* before proceeding, as the default PX4 configuration requires a ground control connection before takeoff. This can be [**downloaded from here (opens new window)**](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html). You also need to load the detector and model parameters. To do this, [download the `VIMU-PythonScript` repo](https://anonymous.4open.science/r/VIMU-PythonScripts-5E80) and [use *QGroundControl* to load the parameter](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/parameters.html) file located in the `data/flight_parameters`.

The drone can then be flown by typing:

```bash
pxh> commander takeoff
```

The drone can be landed by typing `commander land` and the whole simulation can be stopped by doing **CTRL+C** (or by entering `shutdown`).