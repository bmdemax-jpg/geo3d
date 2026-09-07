@echo off
chcp 65001 >nul
title CH05 BLE Bridge
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
python CH05_Bridge.py
echo.
pause
