@echo off
chcp 65001 >nul
title OKM Protocol Sniffer
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
python OKM_Protocol_Sniffer.py
echo.
pause
