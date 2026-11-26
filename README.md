# 2026 FLL Bioglow — Leboblets

PyBricks code for the Leboblets FIRST LEGO League 2026 **Bioglow** robot. The program provides a mission selector, gyro-enabled drivetrain helpers, attachment control, and automated routines for the match missions.

## Robot configuration

| Component | Port |
|---|---|
| Left drive motor | F |
| Right drive motor | E |
| Left attachment motor | D |
| Right attachment motor | C |
| Wheel diameter | 60 mm |
| Axle track | 137 mm |

## Requirements

- LEGO SPIKE Prime or compatible Prime Hub
- PyBricks firmware
- PyBricks development environment or VS Code extension
- Bluetooth connectivity for the hub

## Installation

1. Clone or download this repository:

```bash
git clone https://github.com/mathematicsandscienceisnerd/2026-FLL-Bioglow-Leboblets.git
cd 2026-FLL-Bioglow-Leboblets
```

2. Connect the hub and pair it through your PyBricks environment.

3. Turn on the hub and press its Bluetooth button.

## Running

Open `main.py` in the PyBricks environment and download or run it on the hub.

At startup:

- The hub shows the currently selected mission.
- Left and right buttons change the mission.
- The center button runs the selected mission.
- Green indicates a mission is running.
- Blue indicates the robot is ready after a mission.

## Mission selector

Missions are registered with the `@mission(...)` decorator. Add a new mission by appending a description and function:

```python
@mission("My mission")
def my_mission():
    drive_base.straight(300)
```

### Missions in `main.py`

1. **Drives forward**
2. **Drives backward**
3. **Blue side**
4. **Seed + Rock**
5. **Drone Mission + Animal Drop**
6. **Tree + Vine**

The main copy file contains an earlier blue-side and mission set and is useful for comparing earlier robot paths.

## Repository layout

```text
main.py          Current mission routines and robot control
main copy.py     Earlier backup of the robot program
```

## Notes

- `main.py` calibrates and resets the motors and gyro at startup.
- Movement distances are in millimeters.
- Turn values are in degrees.
- Motor angle values are in degrees.
- The hub stores the mission index between runs.
- Battery voltage and current drive settings are printed to the console.
- Robot-specific measurements and mission coordinates may require retuning after hardware changes.
