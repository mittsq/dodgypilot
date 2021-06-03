![icon partially thanks to FallOutGirl9001 on DeviantArt.](/dodgy_logo.png)
# Welcome to dodgypilot

## What is it and why is it dodgy?
This is an extremely dodgy (No FCW, and driver can press accelerator) fork of openpilot, optimised for a TSS-P Prius with pedal, use with extreme caution.

Panda safety changes were made, do not enable uploader or your device will get banned.

## General information
Fork should only be used on a TSS-P Toyota/Lexus vehicle as there are stock features being replaced with vehicle specific features.

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
Features (branch: 085 or 085-new, depends on which one is updated most recently):
1. No openpilot sounds (Two small chimes from the dash when OP wants driver's attention).
2. No FCW (FCW & AEB handled by Driver Support ECU + SDSU).
3. Includes shamelessly ripped off but modified version of @ShaneSmiskol's comma pedal SnG smoothing code.
4. Includes ripped off but modified version of dragonpilot's "cruise speed override" function.
5. Less acceleration for more eco driving.
6. coloured MPC path (both laned / laneless model) stolen from @kegman.
7. No pings to comma servers.
8. Allows more aggressive braking.
9. Allows driver to use the accelerator.
10. Measurement / Status readings
11. Headlight + Combo Metre based display brightness, similar to vehicle's factory behaviour.
12. Darker UI stolen from @rav4kumar.
13. Keep factory LDA on Toyota/Lexus when openpilot is not enabled.

## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
