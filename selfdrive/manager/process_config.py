import os

from selfdrive.hardware import EON, TICI, PC
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None

procs = [
  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=True, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"], enabled=(not PC or WEBCAM), driverview=True),
  NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"], enabled=False),
  NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], enabled=False),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"]),
  NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], enabled=False),
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=not PC, persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=(not PC or WEBCAM)),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], enabled=False),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("torqued", "selfdrive.locationd.torqued"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enabled=(not PC or WEBCAM), driverview=True),
  PythonProcess("logmessaged", "selfdrive.logmessaged", enabled=False),
  PythonProcess("pandad", "selfdrive.boardd.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=False),
  PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=False),
  PythonProcess("updated", "selfdrive.updated", enabled=False, persistent=False),
  PythonProcess("uploader", "selfdrive.loggerd.uploader", enabled=False),
  PythonProcess("statsd", "selfdrive.statsd", enabled=False),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "selfdrive.hardware.eon.androidd", enabled=EON, persistent=True),

  # Experimental
  PythonProcess("rawgpsd", "selfdrive.sensord.rawgps.rawgpsd", enabled=False), # os.path.isfile("/persist/comma/use-quectel-rawgps")
]

managed_processes = {p.name: p for p in procs}
