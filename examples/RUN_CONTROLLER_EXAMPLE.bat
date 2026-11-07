@ECHO OFF
SETLOCAL

REM Set this to your local PX4PSP/RflySim installation.
IF NOT DEFINED PSP_PATH SET PSP_PATH=C:\PX4PSP

SET PYTHON_EXE=%PSP_PATH%\Python38\python.exe
IF NOT EXIST "%PYTHON_EXE%" SET PYTHON_EXE=python

REM BTS and formation parameters.
SET FORMATION_SPACING=2
SET HOVER_THRUST=0.60
SET MAX_THRUST=0.95
SET STATE_FRAME=true
SET BTS_RHO=0.38
SET BTS_ETA=0.75
SET BTS_DELTA=0.35
SET BTS_K1=0.95
SET BTS_MU=0.65
SET MAX_ACC=5.0
SET FORMATION_TOL=0.30

REM Distributed time-negotiation parameters.
SET TIME_SCALE_XY=80
SET TIME_SCALE_Z=80
SET TIME_CAPABILITY=24
SET TIME_CAPABILITY_FACTORS=1,1,1,1,1,1
SET TIME_DERATING=0.75
SET TIME_BUFFER=8
SET TIME_MIN=10
SET TIME_MAX=30
SET TIME_PSI=0.7
SET TIME_XI=1.5
SET TIME_K_SAFETY=0.8
SET TIME_CONSENSUS_TOL=0.05
SET TIME_CONSENSUS_MAX_TIME=5
SET TIME_RATE_LIMIT=4

REM Initialization and hold-stage parameters.
SET INIT_MIN_TIME=12
SET INIT_POS_TOL=8
SET INIT_VEL_TOL=1.5
SET HOLD_KP=0.65
SET HOLD_KD=1.35

REM Equivalent wind-disturbance parameters.
SET WIND_SEED=260709
SET WIND_LIMIT=1.2
SET WIND_Z_SCALE=0.25
SET TURB_SIGMA=0.30
SET TURB_TAU=1.2
SET TURB_TAU_Z=1.8
SET GUST_RATE=0.14
SET GUST_AMP_MIN=0.40
SET GUST_AMP_MAX=0.85
SET GUST_DURATION_MIN=0.7
SET GUST_DURATION_MAX=1.8
SET GUST_Z_SCALE=0.18

CD /D "%~dp0.."
FOR /F %%i IN ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') DO SET RUN_ID=%%i
SET LOG_DIR=%CD%\logs\run_%RUN_ID%
SET CSV_LOG=%LOG_DIR%\six_circle_bts_wind_px4_att_thrust.csv
SET PLOT_OUT=%LOG_DIR%\six_circle_bts_wind_px4_att_thrust.png
IF NOT EXIST "%LOG_DIR%" MKDIR "%LOG_DIR%"

"%PYTHON_EXE%" -u "%CD%\scripts\six_circle_bts_wind_px4_att_thrust.py" ^
    --side %FORMATION_SPACING% ^
    --rho %BTS_RHO% ^
    --eta %BTS_ETA% ^
    --delta %BTS_DELTA% ^
    --k1 %BTS_K1% ^
    --mu %BTS_MU% ^
    --max-acc %MAX_ACC% ^
    --formation-tol %FORMATION_TOL% ^
    --time-scale-xy %TIME_SCALE_XY% ^
    --time-scale-z %TIME_SCALE_Z% ^
    --time-capability %TIME_CAPABILITY% ^
    --time-capability-factors %TIME_CAPABILITY_FACTORS% ^
    --time-derating %TIME_DERATING% ^
    --time-buffer %TIME_BUFFER% ^
    --time-min %TIME_MIN% ^
    --time-max %TIME_MAX% ^
    --time-psi %TIME_PSI% ^
    --time-xi %TIME_XI% ^
    --time-k-safety %TIME_K_SAFETY% ^
    --time-consensus-tol %TIME_CONSENSUS_TOL% ^
    --time-consensus-max-time %TIME_CONSENSUS_MAX_TIME% ^
    --time-rate-limit %TIME_RATE_LIMIT% ^
    --init-min-time %INIT_MIN_TIME% ^
    --init-pos-tol %INIT_POS_TOL% ^
    --init-vel-tol %INIT_VEL_TOL% ^
    --hold-kp %HOLD_KP% ^
    --hold-kd %HOLD_KD% ^
    --hover-thrust %HOVER_THRUST% ^
    --max-thrust %MAX_THRUST% ^
    --wind-seed %WIND_SEED% ^
    --wind-limit %WIND_LIMIT% ^
    --wind-z-scale %WIND_Z_SCALE% ^
    --turb-sigma %TURB_SIGMA% ^
    --turb-tau %TURB_TAU% ^
    --turb-tau-z %TURB_TAU_Z% ^
    --gust-rate %GUST_RATE% ^
    --gust-amp-min %GUST_AMP_MIN% ^
    --gust-amp-max %GUST_AMP_MAX% ^
    --gust-duration-min %GUST_DURATION_MIN% ^
    --gust-duration-max %GUST_DURATION_MAX% ^
    --gust-z-scale %GUST_Z_SCALE% ^
    --state-frame %STATE_FRAME% ^
    --log "%CSV_LOG%"

IF EXIST "%CSV_LOG%" (
    "%PYTHON_EXE%" "%CD%\scripts\plot_six_circle_trajectory.py" ^
        --csv "%CSV_LOG%" ^
        --out "%PLOT_OUT%"
)

ENDLOCAL

