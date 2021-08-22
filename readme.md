![icon partially thanks to FallOutGirl9001 on DeviantArt.](/dodgy_logo.png)
# Welcome to dodgypilot

## What is it and why is it dodgy?
Dodgypilot is a dodgy fork of comma.ai's openpilot, with forward collision warning disabled, and panda safety altered. Do not enable uploader while using this fork, otherwise your device will be banned.

## General information
Fork should only be used on Toyota/Lexus vehicles as there are stock features being replaced with vehicle specific features.

No feature backports once comma has determined a version to be ready for release, what's on the version stays on that version.
Old versions are left for future references, and should not be used by the end-user.

Branches prefixing "features/" are for cherry-picking. You may cherry-pick features that you like to your fork without going through my code.

## Warnings and ToS
WARNING: openpilot might not compile on your device if I'm doing something to the branch, always wait for the code to stabilise before installing this fork on your device.

By using this software, you agree that:
1. Maintainers of this software, and by definition - their entities, are not responsible for personal injuries, or damages done to your properties (these include your comma device(s) and vehicle(s)) as a result of using this software.
2. You have viewed and acknowledged comma.ai and all its entities' terms of service.

You do everything at your own risk.

## Features
Features (use branch: release2-staging):
1. No openpilot sounds (Car will chime if op wants attention).
2. No FCW (FCW & AEB handled by Driver Support ECU + SDSU).
3. Allow minimum cruise speed override, stolen from DragonPilot.
4. No pings to comma servers.
5. Keep factory LDA on Toyota/Lexus when openpilot is not enabled.
6. Allow accelerator pedal.
7. Measurement / Status readings.
8. Hugely improved display brightness handling.
9. Darker UI stolen from @rav4kumar.
10. ZSS Support

## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
