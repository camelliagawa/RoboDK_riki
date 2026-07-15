@echo off
rem ============================================================
rem  Create desktop shortcuts (run once).
rem  Double-click to create on your Desktop:
rem    record_force  -> record_force.bat  (start force logging)
rem    plot_force    -> plot_force.bat    (plot the latest CSV)
rem  Shortcuts always point to the .bat files in this folder,
rem  so updating/moving the repo keeps them working.
rem ============================================================
set "REPO=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$ws=New-Object -ComObject WScript.Shell; $d=[Environment]::GetFolderPath('Desktop'); $repo=$env:REPO; $map=[ordered]@{'record_force.lnk'=@('record_force.bat','shell32.dll,137'); 'plot_force.lnk'=@('plot_force.bat','shell32.dll,70')}; foreach($k in $map.Keys){ $bat=$map[$k][0]; $t=Join-Path $repo $bat; if(-not (Test-Path $t)){ Write-Host ('skip (not found): '+$bat); continue }; $s=$ws.CreateShortcut((Join-Path $d $k)); $s.TargetPath=$t; $s.WorkingDirectory=$repo; $s.IconLocation=$map[$k][1]; $s.Save(); Write-Host ('created: '+$k) }"
echo.
echo Done. Check your Desktop for: record_force / plot_force
pause
