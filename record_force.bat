@echo off
rem ============================================================
rem  Start force logging with the control panel (RoboDK not required).
rem  Double-click the desktop shortcut to run.
rem   - A panel opens; it does NOT start on its own.
rem   - Press [Start]: it zeroes (tare) then begins logging.
rem     Do NOT touch the tool during "zero measuring".
rem   - Move the robot with the teach pendant while logging.
rem   - Set the overload threshold [N] on screen (Apply / Enter).
rem   - Alarm ON/OFF and Live graph ON/OFF are on-screen toggles
rem     (live graph is OFF by default).
rem   - Press [Stop] to save the CSV; --plot then opens the graph.
rem  Needs matplotlib for the live graph / --plot (pip install matplotlib).
rem  Keep this .bat in the repo folder (next to
rem  force_moment_overlay.py); put its shortcut on the Desktop.
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python force_moment_overlay.py --no-robodk --log --panel --plot"
