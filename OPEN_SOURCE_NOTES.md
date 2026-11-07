# Open-Source Packaging Notes

## Included

- BTS six-UAV HIL controller.
- Log plotting script.
- Figure explanation document.
- Sanitized launch-script examples.

## Excluded

- RflySim/PX4PSP installation files.
- PX4 firmware tree.
- CopterSim binaries.
- QGroundControl binaries.
- Experiment logs and CSV files.
- Screenshots and videos.
- Hardware COM-port mapping.
- Local GPS origin and exact test-site coordinates.
- Personal machine paths beyond generic examples.

## Hardware/Software Assumptions

The released repository is not a standalone simulator. Complete execution requires a full local RflySim/PX4PSP environment, including RflySim, CopterSim, RflySim3D/UE4, QGroundControl, the RflySim Python SDK, MAVLink communication support, and the required external PX4-compatible flight controllers.

The controller expects RflySim's Python SDK to be installed locally and available through `PSP_PATH`.

The default local path used by many RflySim installations is:

```text
C:\PX4PSP
```

Users can override this with:

```bat
SET PSP_PATH=D:\Your\PX4PSP
```

## Recommended Repository Name

```text
negotiated-time-bts-uav-hil
```

## Suggested Citation Placeholder

Add the paper citation here after the manuscript metadata is final.
