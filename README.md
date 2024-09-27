# Artifacts Evaluation Guide

## Download Virtual Machines

- [VIMU VM](https://drive.google.com/file/d/118Sk78zISWRNgpKZb1bsWmwDCUwysrk9/view?usp=sharing).
- [Baseline VM](https://drive.google.com/file/d/1BtuiCU9zZqTv3zz1eGBje_bi8ps0Ll1J/view?usp=sharing).

## Primary Functionality Evaluation

- VIMU
    - Following `VIMU-README.md` on the desktop of VIMU VM to evaluate the primary usage of VIMU against typical sensor attacks.
- Baselines
    - Following `CI-README.md` on the desktop of Baseline VM to evaluate the primary usage of CI against typical sensor attacks.
    - Following `SRR-README.md` on the desktop of Baseline VM to evaluate the primary usage of SRR against typical sensor attacks.
    - Following `SAVIOR-README.md` on the desktop of Baseline VM to evaluate the primary usage of SAVIOR against typical sensor attacks.
    - Following `VIMU-CS-README.md` on the desktop of Baseline VM to evaluate the primary usage of VIMU-CS against typical sensor attacks.
    - Following `SAVIOR-Buffer-README.md` on the desktop of Baseline VM to evaluate the primary usage of SAVIOR-Buffer against typical sensor attacks.
    - Following `VIMU-NoBuffer-README.md` on the desktop of Baseline VM to evaluate the primary usage of VIMU-NoBuffer against typical sensor attacks.

## Download Raw Data for Results Reproduction

- Download the following raw (flight) data INSIDE the VIMU virtual machine.
    - [Fig.5&6&7 Overt Gyro Attacks.zip volume-1](https://drive.google.com/file/d/10MNZyT7W_uZOrnxczbiv6u36_L_6aSg_/view?usp=sharing).
    - [Fig.5&6&7 Overt Gyro Attacks.zip volume-2](https://drive.google.com/file/d/1-JovCd7mFmTeyp_REZ_9TLQBxloxJ1i3/view?usp=sharing).
    - [Fig.5&6&7 Overt Gyro Attacks.zip volume-3](https://drive.google.com/file/d/1-I9mYPKSFyUfXIRhbssPnIQP1WYd6QNT/view?usp=sharing).
    - [Fig.5&6&7 Overt Gyro Attacks.zip volume-4](https://drive.google.com/file/d/1-GqU0fjnemI5_hlXNVMXD7RF0Dy3OVsi/view?usp=sharing).
    - [Fig.5&6&7 Overt Gyro Attacks.zip volume-5](https://drive.google.com/file/d/10JURv1wjKnDvcfgH_BfFfRqVqdNJvjn-/view?usp=sharing).
    - [Fig.8 TTD on GPS-PV.zip](https://drive.google.com/file/d/1WG5ZQ83BodtMzFkDGGxYNC-NG-KrnfKh/view?usp=sharing).
    - [Fig.9 TTD on Stealthy Attacks.zip](https://drive.google.com/file/d/11BsYnSsuGjsFDIapA2Fc5ZilZqY5Yb6M/view?usp=sharing).
    - [Fig.10 Recovery Durations vs. SRR.zip](https://drive.google.com/file/d/1-bqZPEvtn0ozecWNVanuZm-x9rpPLUko/view?usp=sharing).
    - [Fig.11 Recovery Duration (SAVIOR-Buffer vs. VIMU-CS).zip](https://drive.google.com/file/d/1yDUoJdLO7ug6C_STmoQzQPZNmmvgABrz/view?usp=sharing).
    - [Fig.17 SRR Supplementary Compensation.zip](https://drive.google.com/file/d/1GyrpVQizPG35vHG-xc7-G8i4oZH5V8k4/view?usp=drive_link).
    - [Fig.18 Recovery Duration - Effect of Buffer.zip](https://drive.google.com/file/d/10Go_fxDjxlizN8jL6WozplONaq0LLibA/view?usp=sharing).
    - [Fig.19 & Fig.21 Maneuver Gyro Attack.zip](https://drive.google.com/file/d/1-XBKpRrZVWO1zlwX8c1Bt6JfS88B6HtO/view?usp=sharing).
    - [Fig.20 Maneuver Stealthy Attack.zip](https://drive.google.com/file/d/1-cOnDCepehMG83pwfdQB77QxYo0_3-Uo/view?usp=sharing).
- Download the script package INSIDE the VIMU virtual machine for Results Reproduction.
    - [VIMU-FigureScripts.zip](https://drive.google.com/file/d/11GJhhMrtUNpu8OT7E-Sqpw5-TlfEiQsK/view?usp=sharing).
  - NOTE: The following reproducing steps will create a cache folder (located in `VIMU-FigureScripts/figure_data`). It stores the processed data to restore the extraction progress in case of the unwanted interruption (e.g., electrical blackout). The cache can be safely removed once all figures are plotted.
- Reproduce Fig 5, Fig 6, and Fig 7. (Est. Time: 3-6 hours, heavy I/O and computation demand)
  - Unzip `VIMU-FigureScripts.zip` and volumes of `Fig.5&6&7 Overt Gyro Attacks.zip` to the same folder (`'./VIMU-FigureScripts'` and `'./Fig.5&6&7 Overt Gyro Attacks'`).
  - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`.
  - Run `python3 preprocess_flight_logs.py '../Fig.5&6&7 Overt Gyro Attacks' './figure_data/Fig.5&6&7 Overt Gyro Attacks'` to process the raw flight logs. Note that this process would take several hours to finish and will consume **40 GB** to store the processed data. Make sure there is enough space.
  - Run `python3 validate_detector.py './figure_data/Fig.5&6&7 Overt Gyro Attacks'` to generate data for Fig.6.
  - Run `python3 plot_tpr_fpr_heatmap.py` to plot Fig.5 (`TPR_heatmap Overt Gyro Attacks - 4x2.pdf`).
  - Run `python3 plot_roc_curve.py` to plot Fig.6 (`ROC Curves - Gyro.pdf`).
  - Run `python3 plot_ttd_relplot.py` to plot Fig.7 (`Time to Detect - Gyroscope Overt Attack.pdf`).
  - All figure files are located in the script directory.
- Reproduce Fig 8. (Est. Time: < 5min)
    - Unzip `Fig.8 TTD on GPS-PV.zip` and `VIMU-FigureScripts.zip` to the same folder.
  - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`.
    - Run `python3 preprocess_flight_logs.py '../Fig.8 TTD on GPS-PV' './figure_data/Fig.8 TTD on GPS-PV' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_gps_pv_attack_ttd.py` to plot the figure.
    - The figure file `Real GPS Attack TTD.pdf` is located in the script directory.
- Reproduce Fig 9. (Est. Time: 30min with 4 CPU Cores)
  - Unzip `Fig.9 TTD on Stealthy Attacks.zip` and `VIMU-FigureScripts.zip` to the same folder.
  - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`.
  - Run `python3 preprocess_flight_logs.py '../Fig.9 TTD on Stealthy Attacks' './figure_data/Fig.9 TTD on Stealthy Attacks' --skip_threshold_data` to process the raw flight logs.
  - Run `python3 plot_stealthy_attack_chart.py` to plot the figure.
  - The figure file `Stealthy Time to Detect.pdf` is located in the script directory.
- Reproduce Fig 10. (Est. Time: 20~25min with 4 CPU Cores)
    - Unzip `Fig.10 Recovery Durations vs. SRR.zip` and `VIMU-FigureScripts.zip` to the same folder.
    - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`
    - Run `python3 preprocess_flight_logs.py '../Fig.10 Recovery Durations vs. SRR' './figure_data/Fig.10 Recovery Durations vs. SRR' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_recovery_duration_chart.py` to plot the figure.
    - The figure file `Recovery Time Histogram.pdf` is located in the script directory.
- Reproduce Fig 11. (Est. Time: 20~25min with 4 CPU Cores)
    - Unzip `Fig.11 Recovery Duration (SAVIOR-Buffer vs. VIMU-CS).zip` and `VIMU-FigureScripts.zip` to the same folder.
    - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`
    - Run `python3 preprocess_flight_logs.py '../Fig.11 Recovery Duration (SAVIOR-Buffer vs. VIMU-CS)' './figure_data/Fig.11 Recovery Duration (SAVIOR-Buffer vs. VIMU-CS)' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_recovery_duration_with_savior.py` to plot the figure.
    - The figure file `Recovery Duration with SAVIOR.pdf` is located in the script directory.
- Reproduce Fig 17. (Est. Time: < 5min)
    - Unzip `Fig.17 SRR Supplementary Compensation.zip`.
    - Open the terminal console and navigate into the folder with `cd './Fig.17 SRR Supplementary Compensation'`.
    - Run the plotting script with `python3 plot_sup_compensation_error.py`.
    - The figure file `Supplementary Compensation Error.pdf` is located in the script directory.
- Reproduce Fig 18. (Est. Time: 20~25min with 4 CPU Cores)
    - Unzip `Fig.18 Recovery Duration - Effect of Buffer.zip` and `VIMU-FigureScripts.zip` to the same folder.
    - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`
    - Run `python3 preprocess_flight_logs.py '../Fig.18 Recovery Duration - Effect of Buffer' './figure_data/Fig.18 Recovery Duration - Effect of Buffer' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_recovery_duration_to_ttd.py` to plot the figure.
    - The figure file `Effect of Buffer and TTD.pdf` is located in the script directory.
- Reproduce Fig 19 and Fig 21. (Est. Time: 30min with 4 CPU Cores)
    - Unzip `Fig.19 & Fig.21 Maneuver Gyro Attack.zip` and `VIMU-FigureScripts.zip` to the same folder.
    - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`
    - Run `python3 preprocess_flight_logs.py '../Fig.19 & Fig.21 Maneuver Gyro Attack' './figure_data/Fig.19 & Fig.21 Maneuver Gyro Attack' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_recovery_duration_with_savior_maneuver.py` to plot the Fig.19 (`Recovery Duration with SAVIOR - Maneuver.pdf`).
    - Run `python3 plot_ttd_relplot_maneuver.py` to plot the Fig.21 (`Time to Detect - Moving vs Maneuver.pdf`).
  - Both figure files are located in the script directory.
- Reproduce Fig 20. (Est. Time: 10min with 4 CPU Cores)
    - Unzip `Fig.20 Maneuver Stealthy Attack.zip` and `VIMU-FigureScripts.zip` to the same folder.
    - Open the terminal console and navigate into the script folder with `cd './VIMU-FigureScripts'`
    - Run `python3 preprocess_flight_logs.py '../Fig.20 Maneuver Stealthy Attack' './figure_data/Fig.20 Maneuver Stealthy Attack' --skip_threshold_data` to process the raw flight logs.
    - Run `python3 plot_stealthy_attack_chart_maneuver.py` to plot the figure.
    - The figure file `Stealthy Time to Detect - Maneuver.pdf` is located in the script directory.


# PBAR-Enhanced PX4 Drone Autopilot

This repository holds the PBAR-enhanced [PX4](http://px4.io/) flight control solution for drones, with the main applications located in the src/modules directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

## Differences between VIMU-PX4 and the original PX4

The PBAR enhancement modifies codes in the following locations:

- Core: `src/modules/sensors/virtual_imu` (VIMU - Physical Model, EKF, and Buffer Safeguard)
- Sensor Attack & Anomaly Detection
    - `sensor_attack` library: `src/lib/sensor_attack` (Attack Implementation)
    - `fault_detector` library: `src/lib/fault_detector` (Anomaly detector algorithm, VIMU-AD)
    - GPS: `src/modules/sensors/vehicle_gps_position` (GPS Attack & Detector)
    - Barometer: `src/modules/sensors/vehicle_air_data` (Baro Attack & Detector)
    - Magnetometer
        - `src/modules/ekf2` (Detection)
        - `src/modules/sensors/vehicle_magnetometer` (Attack)
    - Accelerometer: `src/lib/drivers/accelerometer` (Accel Attack & Detector)
    - Gyroscope: `src/lib/drivers/gyroscope` (Gyro Attack & Detector)
- Recovery: `src/modules/ekf2` (Recovery Monitor)

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

       Clone any branch in `baseline` (e.g., `baseline/Virtual-IMU` ) to folder `PX4-Autopilot`:

        ```bash
        git clone -b baseline/Virtual-IMU --recursive https://github.com/wangwwno1/Project-VIMU.git PX4-Autopilot
        ```

       Run the **ubuntu.sh** with no arguments (in a bash shell) to install everything:

        ```bash
        bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
        ```

    2. Alternatively, if you just wanted to set up the development environment without getting all the source code, just download **ubuntu.sh** and **requirements.txt** and then run **ubuntu.sh**:

        ```bash
        wget https://raw.githubusercontent.com/wangwwno1/Project-VIMU/baseline/Virtual-IMU/Tools/setup/ubuntu.sh
        wget https://raw.githubusercontent.com/wangwwno1/Project-VIMU/baseline/Virtual-IMU/Tools/setup/requirements.txt
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

The enhanced PX4 source code is stored on Github in the [wangwwno1/Project-VIMU](https://github.com/wangwwno1/Project-VIMU) repository. To get the VIMU-PX4 onto your computer, enter the following command into a terminal:

```bash
git clone -b baseline/Virtual-IMU --recursive https://github.com/wangwwno1/Project-VIMU.git PX4-Autopilot
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

You may need to start *QGroundControl* before proceeding, as the default PX4 configuration requires a ground control connection before takeoff. This can be [**downloaded from here (opens new window)**](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

You also need to load the detector and model parameters. To do this, copy and paste the following commands to the PX4 console.

- Setting parameters for physical model (Required)
    ```bash
    param set IV_IMU_DELAY_US 500000
    param set VIMU_PREDICT_US 20000
    param set VM_ANG_ACC_NOISE 0.075
    param set VM_DRAG_FACTOR 0.05
    param set VM_INERTIA_XX 0.005
    param set VM_INERTIA_XY 0
    param set VM_INERTIA_XZ 0
    param set VM_INERTIA_YY 0.005
    param set VM_INERTIA_YZ 0
    param set VM_INERTIA_ZZ 0.009
    param set VM_LEN_SCALE_X 0.233345
    param set VM_LEN_SCALE_Y 0.233345
    param set VM_LEN_SCALE_Z 1
    param set VM_MASS 0.8
    param set VM_MOTOR_TAU 0.005
    param set VM_THR_FACTOR 4
    param set EKF2_BCOEF_X 45.5
    param set EKF2_BCOEF_Y 45.5
    param set EKF2_MCOEF 0
    ```
- Setting other parameters for flight test (Required)
    ```bash
    param set CBRK_VELPOSERR 0
    param set EKF2_AID_MASK 1
    param set IMU_GYRO_RATEMAX 250
    param set IMU_INTEG_RATE 250
    param set SDLOG_MODE 0
    param set SDLOG_PROFILE 0
    param set MIS_DIST_1WP 2000
    param set MIS_DIST_WPS 2000
    param set CAL_GYRO0_XOFF 0
    param set CAL_GYRO0_YOFF 0
    param set CAL_GYRO0_ZOFF 0
    param set CAL_GYRO1_XOFF 0
    param set CAL_GYRO1_YOFF 0
    param set CAL_GYRO1_ZOFF 0
    param set CAL_GYRO2_XOFF 0
    param set CAL_GYRO2_YOFF 0
    param set CAL_GYRO2_ZOFF 0
    ```
- Setting VIMU's detector parameters (Optional, require restart the simulation)
    ```bash
    param set EKF2_BARO_NOISE 3.5
    param set EKF2_GPS_P_NOISE 0.5
    param set EKF2_GPS_V_NOISE 0.3
    param set EKF2_MAG_NOISE 0.05
    param set IV_ACC_NOISE 0.35
    param set IV_GYR_NOISE 0.1

    param set IV_ACC_ALPHA 0.01
    param set IV_ACC_CSUM_H 3
    param set IV_ACC_EMA_CAP 1.1
    param set IV_ACC_EMA_H 0.95
    param set IV_ACC_MSHIFT 1
    param set IV_BARO_ALPHA 0.05
    param set IV_BARO_CSUM_H 3
    param set IV_BARO_EMA_CAP 0.52
    param set IV_BARO_EMA_H 0.15
    param set IV_BARO_MSHIFT 0.25
    param set IV_GPS_P_ALPHA 0.01
    param set IV_GPS_P_CSUM_H 3
    param set IV_GPS_P_EMA_CAP 0.85
    param set IV_GPS_P_EMA_H 0.45
    param set IV_GPS_P_MSHIFT 0.5
    param set IV_GPS_V_ALPHA 0.01
    param set IV_GPS_V_EMA_CAP 1.1
    param set IV_GPS_V_EMA_H 0.5
    param set IV_GYR_ALPHA 0.01
    param set IV_GYR_CSUM_H 3
    param set IV_GYR_EMA_CAP 0.85
    param set IV_GYR_EMA_H 0.25
    param set IV_GYR_MSHIFT 0.5
    param set IV_MAG_ALPHA 0.01
    param set IV_MAG_CSUM_H 3
    param set IV_MAG_EMA_CAP 0.52
    param set IV_MAG_EMA_H 0.3
    param set IV_MAG_MSHIFT 0.25
    ```
    To suppress the detector, input these command instead:
    ```bash
    param set IV_ACC_CSUM_H 0
    param set IV_ACC_EMA_H 0
    param set IV_BARO_CSUM_H 0
    param set IV_BARO_EMA_H 0
    param set IV_GPS_P_CSUM_H 0
    param set IV_GPS_P_EMA_H 0
    param set IV_GPS_V_EMA_H 0
    param set IV_GYR_CSUM_H 0
    param set IV_GYR_EMA_H 0
    param set IV_MAG_CSUM_H 0
    param set IV_MAG_EMA_H 0
    ```
- Reset the attack parameters to default (disable attack) (Optional)
    ```bash
    param set ATK_ACC_BIAS 0
    param set ATK_APPLY_TYPE 0
    param set ATK_COUNTDOWN_MS 5000
    param set ATK_GPS_P_CAP 10
    param set ATK_GPS_P_CLS 0
    param set ATK_GPS_P_HDG 0
    param set ATK_GPS_P_IV 0.01
    param set ATK_GPS_P_PITCH 0
    param set ATK_GPS_P_RATE 1
    param set ATK_GPS_V_CAP 10
    param set ATK_GPS_V_CLS 0
    param set ATK_GPS_V_HDG 0
    param set ATK_GPS_V_IV 0.01
    param set ATK_GPS_V_PITCH 0
    param set ATK_GPS_V_RATE 1
    param set ATK_GYR_AMP 0
    param set ATK_GYR_FREQ 250
    param set ATK_GYR_PHASE 0
    param set ATK_MULTI_BARO 0
    param set ATK_MULTI_IMU 0
    param set ATK_MULTI_MAG 0
    param set ATK_STEALTH_TYPE 0
    param set IV_DELAY_MASK 0
    param set IV_TTD_DELAY_MS 0
    param set IV_DEBUG_LOG 1
    ```

The drone can then be flown by typing:

```bash
pxh> commander takeoff
```

The drone can be landed by typing `commander land` and the whole simulation can be stopped by doing **CTRL+C** (or by entering `shutdown`).
