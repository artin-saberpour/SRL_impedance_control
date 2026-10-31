# Supernumerary Robotic Limb — Impedance Control

Simulation and visualization of a six-degree-of-freedom (6-DoF) supernumerary robotic limb (SRL) in [MuJoCo](https://mujoco.org/), with an impedance-control implementation, joint-effort logging, and synchronized demonstration videos.

## Demonstration

<img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/c2939f0f-b13e-4a47-9758-30ae4f98ccca" />

*Animated preview. video showing the robot simulation alongside six joint-effort traces.*

## Overview

The project explores impedance-based control of a wearable supernumerary robotic limb in simulation. The MuJoCo model contains a six-joint robot and an end effector. The simulation can be inspected interactively, while a separate recording workflow combines the robot motion with time-aligned plots of joint efforts.

**Included workflows**

- Load and simulate the SRL model in MuJoCo.
- Execute the controller in `main2.py`.
- Visualize the model in MuJoCo's passive viewer.
- Record six per-joint generalized-force channels against simulation time.

## Requirements

- Python 3.11 (the macOS configuration tested during development)
- MuJoCo 3.x (used with 3.10.0)
- NumPy, SciPy, Matplotlib
- Dynamixel SDK if the hardware interface is imported
- OpenCV (headless) and `imageio-ffmpeg` for demo generation

Install the key dependencies in your active Python environment:

```bash
python -m pip install mujoco numpy scipy matplotlib dynamixel-sdk \
  opencv-python-headless imageio-ffmpeg
```

Additional packages may be required by your particular `main2.py` and `hw_interface.py`. Install them according to the imports in those files.

## Running the simulation

On **macOS**, launch the MuJoCo passive viewer using `mjpython` rather than `python`:

```bash
MPLBACKEND=Agg mjpython main2.py
```

`MPLBACKEND=Agg` prevents Matplotlib GUI windows from being opened on non-main threads. It does **not** display interactive torque plots; use the demo/plot workflow below instead.

On other platforms, MuJoCo's regular Python launcher may be sufficient:

```bash
MPLBACKEND=Agg python main2.py
```

**Hardware caution:** The controller imports `hw_interface.py`, which uses the Dynamixel SDK. Check whether your version performs hardware initialization or sends commands before running with physical motors connected. The instructions above do not guarantee simulation-only execution.

