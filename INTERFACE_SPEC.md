# Interface Specification

This document describes the public interfaces exposed by this repository.

## 1. Controller Script

File:

```text
scripts/six_circle_bts_wind_px4_att_thrust.py
```

### Purpose

Six-UAV distributed BTS formation controller for RflySim/PX4 HIL.

### Main Responsibilities

- Connect to six PX4/CopterSim MAVLink endpoints.
- Initialize the UAVs at predefined starting positions.
- Compute distributed time-consensus states.
- Generate BTS acceleration commands.
- Convert acceleration to PX4 attitude quaternion and normalized thrust.
- Write runtime CSV logs.

### Key Runtime Inputs

The script accepts command-line arguments. The most important ones are:

- `--ip`
- `--start-id`
- `--rate`
- `--duration`
- `--T`
- `--manual-time`
- `--side`
- `--eta`
- `--delta`
- `--rho`
- `--k1`
- `--mu`
- `--max-acc`
- `--init-kp`
- `--init-kd`
- `--init-min-time`
- `--init-pos-tol`
- `--init-vel-tol`
- `--hold-kp`
- `--hold-kd`
- `--formation-tol`
- `--time-scale-xy`
- `--time-scale-z`
- `--time-capability`
- `--time-capability-factors`
- `--time-derating`
- `--time-buffer`
- `--time-min`
- `--time-max`
- `--time-psi`
- `--time-xi`
- `--time-k-safety`
- `--time-consensus-tol`
- `--time-consensus-max-time`
- `--time-rate-limit`
- `--hover-thrust`
- `--min-thrust`
- `--max-thrust`
- `--wind-amp`
- `--wind-freq`
- `--wind-z-scale`
- `--wind-seed`
- `--wind-limit`
- `--turb-sigma`
- `--turb-tau`
- `--turb-tau-z`
- `--gust-rate`
- `--gust-amp-min`
- `--gust-amp-max`
- `--gust-duration-min`
- `--gust-duration-max`
- `--gust-z-scale`
- `--yaw`
- `--state-frame`
- `--log`

### Main Outputs

- PX4 attitude quaternion and thrust commands
- CSV log file
- runtime console status

### Log Fields

The CSV log contains:

- `t`
- `stage`
- `spread`
- `formation_error`
- `center_x`
- `center_y`
- `center_z`
- `negotiated_T`
- `time_disagreement`
- per-UAV position, velocity, attitude, offsets, wind, time-state columns

## 2. Launch Script Templates

### `examples/START_HITL_EXAMPLE.bat`

Purpose:

- Start QGroundControl
- Start RflySim3D
- Start six CopterSim instances
- Prompt the user for six COM ports
- Launch the six-UAV controller

### `examples/RUN_CONTROLLER_EXAMPLE.bat`

Purpose:

- Run the controller with parameterized BTS, time-consensus, and wind settings
- Save logs to a timestamped `logs/run_YYYYMMDD_HHMMSS` directory
- Generate post-processing figures automatically

## 3. Plotting Script

File:

```text
scripts/plot_six_circle_trajectory.py
```

### Purpose

Convert CSV logs into publication-style figures.

### Generated Figures

- `*_3d.png`
- `*_xy.png`
- `*_closure_rings.png`
- `*_error.png`
- `*_xyz.png`
- `*_state3d.png`
- `*_state_xyz.png`
- `*_time_consensus.png`
- `*_attitude.png`

## 4. External Dependencies

This repository assumes the following are installed locally and are not redistributed here:

- RflySim SDK
- PX4PSP
- CopterSim
- QGroundControl
- six PX4-compatible flight controllers

The controller imports:

```text
PX4MavCtrlV4
```

from the local RflySim SDK path.

## 5. Configuration Summary

Editable experiment settings are intentionally kept as command-line arguments or batch variables.

Do not hard-code:

- local COM-port mappings
- site-specific GPS origins
- machine-specific install paths
- private experiment logs
