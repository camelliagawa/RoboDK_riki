@echo off
rem ============================================================
rem  Plot the latest grind log in the WHEEL frame
rem  (Fn = push, Ft = drag (reference), Fa = along blade).
rem  The usual plot_sides.bat is unchanged.
rem
rem  Put these in the same folder as the log (or in a folder
rem  named like the log, e.g. force_log_20261028_155814\):
rem    HaR.LS / HaL.LS / kenma.LS  (the LS used for that grind)
rem    air.csv                      (air run WITH the knife)
rem  If the LS files are missing, the normal plot opens instead.
rem
rem  This launcher auto-updates to the latest version (git pull).
rem  NOTE: keep this file ASCII-only.
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; $env:GIT_TERMINAL_PROMPT=0; Write-Host 'Updating to latest (git pull)...'; git pull; python world_frame.py --frame wheel %*"
