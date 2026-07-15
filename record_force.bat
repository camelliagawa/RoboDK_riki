@echo off
rem ============================================================
rem  Start force logging (RoboDK not required).
rem  Double-click the desktop shortcut to run.
rem   - Do NOT touch the tool during "zero measuring" (tare).
rem   - Move the robot with the teach pendant while logging.
rem   - Press Ctrl+C in this window to stop and save the CSV.
rem  Keep this .bat in the repo folder (next to
rem  force_moment_overlay.py); put its shortcut on the Desktop.
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python force_moment_overlay.py --no-robodk --log"
