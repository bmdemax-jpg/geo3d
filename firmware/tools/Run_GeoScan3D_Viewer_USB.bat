@echo off
chcp 65001 >nul
title GeoScan3D Viewer (USB)
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
python GeoScan3D_Viewer_USB.py
echo.
pause
