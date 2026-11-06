@echo off
chcp 65001 >nul
cd /d "%~dp0"
start "" "%~dp0reports\observation_v1\val\review.html"
if exist "%~dp0reports\observation_v1\benchmark\review.html" start "" "%~dp0reports\observation_v1\benchmark\review.html"
if exist "%~dp0dataset_observation_v3\review.html" start "" "%~dp0dataset_observation_v3\review.html"
