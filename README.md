# Toilet World Simulation - Gazebo (Ignition Gazebo / gz sim)

This repository contains a bathroom/toilet simulation environment for use with Gazebo (Ignition Gazebo, `gz sim`).

## Folder Structure

- `toilet_world.world` - Main world file for simulation
- `modified.sdf` - Alternate SDF world file
- `models/` - Contains all required model folders (toilet, bathroom_sink, Floor_room, etc.)

## Prerequisites
- **Gazebo (Ignition Gazebo / gz sim)** installed (tested with Fortress, Harmonic, or later)
- Linux OS (tested on Ubuntu)

## How to Run the Simulation

1. **Clone this repository:**
   ```bash
   git clone <repo_url>
   cd <repo_folder>
   ```

2. **Launch the world:**
   ```bash
   export GZ_SIM_RESOURCE_PATH="/absolute/path/to/models"
   gz sim /absolute/path/to/world_file.sdf
   ```

## Troubleshooting
- If you see errors like `Unable to find or download file`, ensure:
  - The `GZ_SIM_RESOURCE_PATH` is set to the absolute path of the `models` directory.
  - All model folders and files exist and are readable.
  - Folder names in `models/` match the names used in the world file `<uri>model://...` tags.
- If using Snap or Flatpak installations of Gazebo, you may encounter library issues. Prefer native package or source installations for best compatibility.

## Credits
- Model sources and world design inspired by AWS RoboMaker and open-source Gazebo model datasets.

---
For questions or issues, please open an issue in this repository.
