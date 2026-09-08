@echo off
chcp 65001 >nul
title GeoScan3D Viewer
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
python GeoScan3D_Viewer.py
echo.
pause
