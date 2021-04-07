**Welcome to dodgypilot**

This is an extremely dodgy (No FCW and driver can press accelerator) fork of openpilot, optimized for a TSS-P Prius with pedal, use with extreme caution.

You should use this fork if you are looking forward to getting banned.

You should only use this fork on a Toyota/Lexus vehicle as there are stock features being replaced with vehicle specific features.

Features (tested on Toyotas only) (branch: 084 or 084-new, depends on which one is updated most recently):
1. No sound (Not yet complete, car chimes on disengage, this can be fixed if op does not send `pcm_cancel` command)(Two small chimes from the dash when OP wants your attention).
2. No FCW (FCW & AEB handled by Driver Support ECU + SDSU).
3. Toyota/Lexus hybrids with pedal only - Includes shamelessly ripped off but modified version of @ShaneSmiskol's comma pedal SnG smoothing code (Extremely smooth stop and go, almost no double braking, capable of very low speed pedal ACC with very little jerking).
4. Includes shamelessly ripped off but modified version of dragonpilot's "cruise speed override" function.
5. Less acceleration for more eco driving.
6. Brake disc icon, ~~coloured MPC path~~ (removed until code stabilises) shamelessly copied from @kegman
7. No uploading.
8. Toyota/Lexus only - Allows hard braking (can stop in time for a stopped car when travelling at 80 km/h).
9. Toyota/Lexus only - Allows driver to use the accelerator.
10. Toyota/Lexus hybrids only - HV ICE RPM and acceleration command readings, with bb measurement readings stolen from @kegman
11. Toyota/Lexus only - Headlight based display brightness, similar to your car's factory dash, no comma brightness control, NO MORE BLINDING.
12. Dark UI stolen from @rav4kumar

