# Fork Changes
This fork (mittsq) changes the following four things:
* Changes git targets to this repository
* Changes the logo to not a doge
* Adds nudgless lane changes starting at 52 mph
* Makes the speed turn red on brake instead of using an emoji
* Adds the stop-and-go hack

Everything else is unchanged from the [parent repository](https://github.com/cydia2020/dodgypilot).
Since there is no release branch, install `c2` instead; it will compile on-device.

---

![icon partially thanks to FallOutGirl9001 on DeviantArt.](/dodgy_logo.png)
# Welcome to dodgypilot

## What is this?
Dodgypilot is a dodgy fork of comma.ai's [openpilot](https://openpilot.comma.ai). It is actively maintained by cydia2020.

## General information
This fork has a vehicle whitelist, and will only run on a Toyota/Lexus vehicle.

*Please use the precompiled branch "release2-staging", this branch has been tested to function in normal circumstances.*

No feature backports once comma has determined a version to be ready for release, what's on the version stays on that version.
Old versions are left for future references, and should not be used by the end-user.

## What about the comma three?
The maintainer of this fork does not currently own a comma three and is not planning on getting one for the foreseeable future. This fork will be stuck on 0.8.14 for the time being. Feel free to adopt the changes to your own fork.

## Installation
Dodgypilot can only be installed after you have enabled ssh on your device.

To install this fork, simply ssh into your comma device, `cd /data/openpilot`, `git remote add cydia2020 https://github.com/cydia2020/dodgypilot`, then `git checkout cydia2020/release2-staging`.

## Preparing Your Vehicle
‼️ READ THIS BEFORE INSTALLING ‼️

IMPORTANT: You must turn off the vehicle's stock Steering Assist feature for dodgypilot to engage. To turn Steering Assist off, go to your car's settings screen, highlight the steering assist toggle, then press `OK` once to toggle it off. Toyota has a [helpful guide](https://www.youtube.com/watch?v=qEvAua6oobA) on this.

If you have a TSS-P Vehicle, it is recommended that you put together or buy a SmartDSU from Taobao or @ErichMoraga. It will lessen your chance of crashing while you are using dodgypilot. A SmartDSU also enables features such as follow distance adjustment and switching between openpilot and stock longitudinal.

## Warnings and ToS
WARNING: openpilot might not compile on your device if I'm doing something to a non-precompiled branch, always wait for the code to stabilise before installing this fork on your device.

By using this software, you agree that:
1. Maintainers of this software, and by definition - their entities, are not responsible for personal injuries, or damages done to your properties (these include your comma device(s) and vehicle(s)) as a result of using this software.
2. You have viewed and acknowledged comma.ai and all its entities' terms of service.

You do everything at your own risk.

## Features
This fork:
1. Disables openpilot sounds. (Car will chime differently based on the severity of the alert if dodgypilot wants attention. openpilot sounds can be re-enabled by a toggle in the comma device's settings)
2. Supports ZSS.
3. Disables the uploader. (UI API still active to maintain ease of maintenance)
4. Keeps factory LDA and SWS on Toyota/Lexus.
5. Displays radarState readings on the onroad UI.
6. Improves screen brightness handling by linking it with your headlights. To use this feature, go to settings, and turn on `Use Linked Brightness`.
7. Allows openpilot to be engaged even if adaptive cruise control is disabled. To use this feature, go to settings, and turn on `Allow Normal Cruise Control`.
8. Allows switching between openpilot Long and stock ACC, requires SmartDSU. To use this, go to settings, and turn on `Use Stock ACC`.
9. Does not clear car parameters on startup, this improves startup speed and helps with params faults on startup. To reset car parameters, go to settings, and press `RESET CAR RECOGNITION`.
10. Improves Toyota/Lexus lateral control with the new torque controller. (please open an issue if your car doesn't work well with this. Torque controller can be disabled by a toggle in your comma device's settings.)
11. Improves stop-and-go performance, reduce acceleration oscillation.
12. Allows low cruise speed override (stolen from DragonPilot). To use this, ensure that you have a vehicle that's capable of openpilot longitudinal control, go to settings, and enable `Cruise Speed Override`. When engaging under approximately 43.2km/h (45-46 km/h on the HUD), dodgypilot will set its cruise speed to the vehicle's current travel speed.
13. Allows the driver to change openpilot's follow distance on Toyota/Lexus with openpilot longitudinal control (and SmartDSU if the user has a TSS-P vehicle, please use [this](https://github.com/wocsor/panda/commit/0c10024d5250c737d5ae6b00f8d7c3341896b71f) firmware for your SmartDSU). (Stolen from @krkeegan)(Image below is an aRTiSt's iMprEsSioN)
![Distance Indicator](/follow_distance_indicator.png)
14. Respects the Powertrain Control Module, and does not cancel cruise control when it is not necessary. (e.g. brake press)
15. Allows the device's display to be turned off after 60 seconds of inactivity (no touch input). (Stolen from @sunnyhaibin)
16. Includes a vehicle selection feature, allows users without access to a comma power to set the car model manually. (stolen from @sunnypilot) (Settings -> Fingerprint)

## Support dodgypilot
Thank you for using dodgypilot, if you would like to buy me a coffee, please use the link below.

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/donate/?business=ZE32GX6TZNMCG&no_recurring=1&item_name=Buy+me+a+coffee+and+support+the+development+and+maintenance+of+dodgypilot.&currency_code=AUD)

## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
