# ICR System (Instantaneous Cue Rotation)

Minimal MATLAB code used to run the ICR arena task, which rotates all orienting visual cues in real time while rats navigate a circular track. The setup tests allothetic vs. idiothetic inputs without removing the animal from the arena.

## Quick start

From MATLAB:

```matlab
% status = ICR_GUI(SYSTEST, BREAKDEBUG, DOAUTOLOAD, DOPROFILE, ISMATSOLO);

% Typical debug run (normal, break-on-error, autoload defaults):
status = ICR_GUI(0, 1, true, false, false);
```

**Args**

* `SYSTEST` `[0..8]`: 0 normal | 1 sim rat | 2 PID calib | 3 VT calib | 4 halt-error | 5 wall-image IR sync | 6 IR sync | 7 hardware test | 8 cube battery
    
* `BREAKDEBUG`: 0 no break | 1 break on errors | >1 break at line
    
* `DOAUTOLOAD`: `true|false` autoload hardcoded rat/session values
    
* `DOPROFILE`: `true|false` enable MATLAB profiler
    
* `ISMATSOLO`: `true|false` MATLAB running alone or with other programs
    

Requirements
------------

* MATLAB (tested historically with R2018+)
    
* Optional: Neuralynx Cheetah for TTL I/O and timestamping
    

What it does
------------

* Initializes globals and runtime state, then calls `Setup() → Run() → Exit()` with error-aware control flow
    
* Provides system tests for simulated rat, calibration, sync timing, and hardware checks
    
* When present, identifies to Cheetah, verifies timestamp/connection, and configures TTL I/O (sound, reward, PID state)
    

Key session parameters (autoload defaults example)
--------------------------------------------------

```matlab
D.DB.Session_Type       = 'ICR_Session';         % ['ICR_Session' 'TT_Turn' 'Table_Update']
D.DB.Session_Condition  = 'Behavior_Training';   % e.g., 'Rotation', 'Dark_Control'
D.DB.Session_Task       = 'Track';               % ['Track' 'Forage']
D.DB.Feeder_Condition   = 'C2';                  % mobile feeder option
D.DB.Reward_Delay       = '3.0';                 % seconds
D.DB.Cue_Condition      = 'None';                % ['All' 'Half' 'None']
D.DB.Rotation_Direction = 'CCW';                 % ['CCW' 'CW']
D.DB.Start_Quadrant     = 'NW';                  % ['NE' 'SE' 'SW' 'NW']
D.DB.Rotation_Positions = [180,180,180,90,180,270,90,180,270];
```

Apparatus summary
-----------------

The ICR apparatus projects a 360° panorama onto arena walls and can rotate all orienting cues instantaneously while the rat is running. This creates immediate conflict between visual and self-motion signals without disturbing vestibular feedback; navigation accuracy is read out via a cue-aligned goal task. A mobile robotic feeder minimizes local olfactory cues.

Citation
--------

If you use this code or task design, please cite:

Lester AW, Kapellusch AJ, Barnes CA (2020). _A novel apparatus for assessing visual cue-based navigation in rodents_. **Journal of Neuroscience Methods**, 338:108667.

Notes
-----

* Default autoload rat/session IDs and flags are set near the top of `ICR_GUI.m`
    
* Use system tests for quick hardware/sync sanity checks before real sessions