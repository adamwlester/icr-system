ICR System (Instantaneous Cue Rotation)
=======================================

Software for the Instantaneous Cue Rotation arena: controls panoramic cue rotations, drives a mobile feeder robot, streams position, and synchronizes events for closed-loop rodent navigation experiments. This README is recruiter-friendly and focuses on the codebase.

Highlights
----------

* Real-time cue switching and projector orchestration.
    
* Mobile feeder robot control with closed-loop position and reward delivery.
    
* Robust comms path (MATLAB → C# service → Arduino Due/XBee).
    
* Deterministic timing with IR/TTL sync and mirrored robot/PC logging.
    
* Turnkey .mat inputs and outputs for control and analysis.
    

Tech Stack
----------

* **MATLAB (R2017a+)** UI, task logic, projector control, analysis utils
    
* **C# (.NET)** serial/XBee transport, packet framing, stream server
    
* **Arduino/C++ (Due/SAM3X8E)** stepper/PID, Pixy CMUcam5, XBee, SD, IR sync
    

Repository Structure
--------------------

```
icr-system/
├─ csharp/                      – C# application for Windows that orchestrates the experiment.
│   ├─ ICR_Run.cs               – main control program.
│
├─ arduino/
│   ├─ CheetahDue/              – firmware for fixed feeder and synchronisation controller.
│   │   ├─ CheetahDue.ino       – main Arduino sketch.
│   │   ├─ CheetahDue.h         – class definitions and constants.
│   │   └─ CheetahDue_PinMap.h  – pin mappings for TTL outputs and sensors.
│   ├─ FeederDue/               – firmware for mobile feeder cart.
│   │   ├─ FeederDue.h          – main firmware (header-only implementation).
│   │   └─ FeederDue_PinMap.h   – pin mappings for motor drivers, solenoids and sensors.
│
├─ matlab/
│   ├─ support_code/dlgAWL/     – generic file and dialog utilities used by the GUI.
│   ├─ ICR_GUI.m                – primary GUI and control code.
│   └─ AC_NETMON.m              – image and sound display code for projector computer.
│
├─ data/                        – experiment resources, Neuralynx configs, images, and session data.
│   ├─ cheetah/                 – Neuralynx (Cheetah/SpikeSort3D) configuration files.
│   │   └─ configuration/       – .cfg files for behavior and ephys setups (requires manual path edits).
│   ├─   images/                – visual assets for image projection.
│   │   ├─ icons/               – small UI and GUI icons.
│   │   ├─ paxinos/             – Paxinos brain atlas reference images (coronal, sagittal, horizontal).
│   │   └─ wall_images/         – cue wall textures used during behavior (main, testing, trigger sets).
│   ├─ io_file_setup/           – MATLAB scripts to configure IO .mat session files.
│   ├─ operational/             – precomputed path and track-boundary files for tasks.
│   └─ session/                 – example session-level IO .mat files for saved task configurations.
│
│─ testing/
│   └─ HardwareDiagnostic/      – Arduino sketch for verifying TTL lines, solenoids and sensors.
└─  config.json                 – global path configuration file used by both MATLAB and C#.
```

Core Capabilities (Code-Level)
------------------------------

* **Cue control** Atomic swaps between pre-warped panoramas; phototransistor pulses mark each change.
    
* **Robot control** Velocity/position loops keep an offset to the animal; reward arm and solenoids actuated with safety states.
    
* **Sensor fusion** Overhead camera + Pixy fused on-robot via EKF for smooth pose/velocity.
    
* **Sync & logging** IR sync with PC TTL for sub-frame alignment; robot SD logs mirrored to PC.
    
* **Reliability** Watchdogs, heartbeats, bounded queues, and fail-safe motor/solenoid states.
    

Path Config Setup
------------

* **Paths** Define global paths in `config.json` at the repo root. Both MATLAB and C# code read this file instead of using hardcoded directories.
* **Cheetah configs** Certain paths still need to be manually updated in the attached Neuralynx configuration files:
  * `data\cheetah\configuration\ICR_Basic.cfg`
  * `data\cheetah\configuration\Cheetah.cfg`

Quick Start
-----------

1. **Firmware** Flash `arduino/FeederDue` to an Arduino Due; configure XBee, motor drivers, solenoids, Pixy.
    
2. **Comms service** Build and run `csharp/` on Windows; select the XBee COM port.
    
3. **MATLAB** Open the control app, point to projector assets and IO/COM paths, connect to the C# service.
    
4. **Run** Start a session; MATLAB drives cues, C# relays commands and streams position, the robot executes motion and rewards.
    
5. **End** MATLAB writes session CSVs; robot streams SD logs for reconciliation.
    

Outputs
-------

* **PC** CSV/TSV for position, cue state, rewards, and TTL events.
    
* **Robot** SD logs with local timestamps; merged during post-session reconciliation.
    

Testing
-------

* Dry-run modes disable solenoids and limit motor torque for benchtop checks.
    
* `testing/HardwareDiagnostic` verifies TTL lines, solenoids, sensors, and radio.
    

License
-------

See `LICENSE`.

Citation
--------

Lester, A. W., Kapellusch, A. J., & Barnes, C. A. (2020). A novel apparatus for assessing visual cue-based navigation in rodents. _Journal of Neuroscience Methods_, 338, 108667.

* * *

Summary: I delivered a concise, recruiter-friendly Markdown README with an industry-standard structure and your requested folder tree, focusing on the mobile feeder robot and codebase capabilities, setup, runtime flow, outputs, and testing.