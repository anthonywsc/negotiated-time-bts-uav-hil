# Contributing

This repository contains research code for a RflySim/PX4 HIL experiment. Contributions should keep the code reproducible and avoid committing local machine configuration.

## Before Opening a Pull Request

1. Do not commit logs, screenshots, videos, `.pyc` files, or local hardware mappings.
2. Do not commit private GPS coordinates, COM-port lists, or personal paths.
3. Keep RflySim/PX4PSP SDK dependencies external. Do not vendor proprietary or third-party binaries here.
4. Run a Python syntax check:

```bat
python -m py_compile scripts\six_circle_bts_wind_px4_att_thrust.py scripts\plot_six_circle_trajectory.py
```

## Code Style

- Keep controller changes explicit and parameterized.
- Prefer adding command-line arguments instead of hard-coding local experiment settings.
- Keep plotting changes reproducible from CSV logs.

