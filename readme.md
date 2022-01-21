### not maintained right now as I have no access to a working device, maintenance efforts will resume once I get my device fixed.

![icon partially thanks to FallOutGirl9001 on DeviantArt.](/dodgy_logo.png)
# Welcome to dodgypilot

## What is this?
Dodgypilot is a dodgy fork of comma.ai's [openpilot](https://openpilot.comma.ai). It is actively maintained by cydia2020.

## General information
This fork has a vehicle whitelist, and will only run on a Toyota/Lexus vehicle.

*Please use the precompiled branch "release2-staging", this branch has been tested to function in normal circumstances.*

No feature backports once comma has determined a version to be ready for release, what's on the version stays on that version.
Old versions are left for future references, and should not be used by the end-user.

## Installation
Dodgypilot can only be installed after you have enabled ssh on your device.

To install this fork, simply ssh into your comma device, `cd /data/openpilot`, `git remote add cydia2020 https://github.com/cydia2020/dodgypilot`, then `git checkout cydia2020/release2-staging`.

## Prepare Your Vehicle
If you have a TSS-P Vehicle, it is recommended that you put together or buy a SmartDSU from Taobao or @ErichMoraga. It will lessen your chance of crashing while you are using dodgypilot.

Dodgypilot forwards the car's camera message when it is not engaged, as a result, you may experience brief "hands on wheel" warnings from the car's dash when you disengage dodgypilot.

To prevent this, go to your car's settings and disable "LDA Steering Assist".

This will not affect the functionalities of dodgypilot, dodgypilot will turn Steering Assist on when it needs to.

## Warnings and ToS
WARNING: openpilot might not compile on your device if I'm doing something to a non-precompiled branch, always wait for the code to stabilise before installing this fork on your device.

By using this software, you agree that:
1. Maintainers of this software, and by definition - their entities, are not responsible for personal injuries, or damages done to your properties (these include your comma device(s) and vehicle(s)) as a result of using this software.
2. You have viewed and acknowledged comma.ai and all its entities' terms of service.

You do everything at your own risk.

## Features
This fork:
1. Disables openpilot sounds (Car will chime if op wants attention.).
2. Supports ZSS.
3. Disables the uploader. (UI API still active to maintain ease of maintenance.)
4. Keeps factory LDA on Toyota/Lexus when openpilot is not enabled.
5. Allows the driver to use the accelerator pedal. To use this, go to settings, and turn on Allow Gas Pedal.
6. Displays radarState, ~~carState and deviceState~~ readings on the onroad UI. (Partially removed for now, I'll port this back once I'm familiar enough with Qt.)
7. Improves screen brightness handling by linking it with your headlights. To use this feature, go to settings, and turn on `Use Linked Brightness`.
8. Allows openpilot to be engaged even if adaptive cruise control is disabled. To use this feature, go to settings, and turn on `Allow Normal Cruise Control`.
9. Allows switching between openpilot Long and stock ACC, requires SmartDSU. To use this, go to settings, and turn on `Use Stock ACC`.
10. Allows minimum cruise speed override, down to 40km/h, stolen from DragonPilot. (Must not be used together with 9.)
11. Does not clear car parameters on startup, this improves startup speed and helps with params faults on startup. To reset car parameters, go to settings, and press `RESET CAR RECOGNITION`.


## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
