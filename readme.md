![icon partially thanks to FallOutGirl9001 on DeviantArt.](/dodgy_logo.png)
# Welcome to dodgypilot

## What is it and why is it dodgy?
Dodgypilot is a dodgy fork of comma.ai's openpilot, with forward collision warning disabled, and panda safety altered. Do not enable uploader while using this fork, otherwise your device will be banned.

## General information
Fork should only be used on Toyota/Lexus vehicles as there are stock features being replaced with vehicle specific features.

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
1. No device sounds (Car will chime if op wants attention.).
2. ZSS Support.
3. No uploader. (UI API still active to maintain ease of maintenance.)
4. Keeps factory LDA on Toyota/Lexus when openpilot is not enabled.
5. Allows the driver to use the accelerator pedal. To use this, go to settings, and turn on Allow Gas Pedal.
6. Onroad UI Displays radarState, ~~carState and deviceState readings.~~ Partially removed for now, I'll port this back once I'm familiar enough with Qt.
7. Toyota only: Improved screen brightness handling by linking it with your headlights. To use this, go to settings, and turn on Brightness linking.
8. Darker UI stolen from @rav4kumar.
9. Allows switching between openpilot Long and stock ACC, requires SmartDSU. To use this, go to settings, and turn on Stock ACC.
10. Allows minimum cruise speed override, down to 40km/h, stolen from DragonPilot. (Must not be used together with 9.)


## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
