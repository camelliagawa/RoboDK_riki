@echo off
rem ============================================================
rem  デスクトップにショートカットを作る (最初に一度だけ実行)
rem  このファイルをダブルクリックすると、デスクトップに
rem    record_force  … 力の記録を開始 (record_force.bat)
rem    plot_force    … 記録CSVをグラフ表示 (plot_force.bat)
rem  の2つのショートカットが作られます。
rem  ショートカットは常にこのフォルダの .bat を指すので、
rem  リポジトリを移動/更新してもそのまま使えます。
rem ============================================================
set "REPO=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$ws=New-Object -ComObject WScript.Shell; $d=[Environment]::GetFolderPath('Desktop'); $repo=$env:REPO; $map=[ordered]@{'record_force.lnk'=@('record_force.bat','shell32.dll,137'); 'plot_force.lnk'=@('plot_force.bat','shell32.dll,70')}; foreach($k in $map.Keys){ $bat=$map[$k][0]; $t=Join-Path $repo $bat; if(-not (Test-Path $t)){ Write-Host ('skip (not found): '+$bat); continue }; $s=$ws.CreateShortcut((Join-Path $d $k)); $s.TargetPath=$t; $s.WorkingDirectory=$repo; $s.IconLocation=$map[$k][1]; $s.Save(); Write-Host ('created: '+$k) }"
echo.
echo Done. Check your Desktop for: record_force / plot_force
pause
