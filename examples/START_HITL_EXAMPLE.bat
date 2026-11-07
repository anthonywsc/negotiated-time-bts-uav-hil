@ECHO OFF
SETLOCAL EnableDelayedExpansion

REM This is a sanitized template. Edit the placeholders before use.
IF NOT DEFINED PSP_PATH SET PSP_PATH=C:\PX4PSP

SET /A START_INDEX=1
SET /A CLASS_3D_ID=-1
SET DLLModel=0
SET SimMode=0
SET /A BaudRate=921600
SET UE4_MAP=Grasslands
SET IS_BROADCAST=0
SET UDPSIMMODE=Mavlink_Full
SET /A IsSysID=0
SET /A IsAltAng=0
SET isPosGps=1
SET isBatLLAOrin=1

REM Replace this with your own non-private test origin.
SET LatLongAlt=LATITUDE,LONGITUDE,ALTITUDE

REM Replace these with six GPS spawn points or switch to local coordinate spawning.
SET PosXStr=LAT1,LAT2,LAT3,LAT4,LAT5,LAT6
SET PosYStr=LON1,LON2,LON3,LON4,LON5,LON6
SET YawStr=0,0,0,0,0,0

ECHO === Six-UAV USB HIL startup template ===
ECHO Edit LatLongAlt, PosXStr, PosYStr, and COM ports before use.
ECHO.

FOR /F "delims=" %%t IN ('"%PSP_PATH%\CopterSim\GetComList.exe" 3 %BaudRate%') DO SET ComInfoStr=%%t
FOR /F "tokens=1,2,3 delims=#" %%a IN ("%ComInfoStr%") DO (
    SET ComNumExe=%%a
    SET ComNameList=%%b
    SET ComInfoList=%%c
)

ECHO All COM ports detected:
ECHO %ComInfoList%
IF NOT "%ComNameList%"=="" ECHO Suggested COM list: %ComNameList%
SET /P ComNum=Input six COM numbers, comma separated:

SET string=%ComNum%
SET /A VehicleNum=0
:COUNT_COM
FOR /F "tokens=1,* delims=," %%i IN ("!string!") DO (
    SET subStr=%%i
    SET string=%%j
)
IF "!subStr!"=="" GOTO COUNT_DONE
SET /A VehicleNum=!VehicleNum!+1
IF NOT "!string!"=="" GOTO COUNT_COM

:COUNT_DONE
IF NOT "%VehicleNum%"=="6" (
    ECHO ERROR: Please input exactly six COM numbers.
    PAUSE
    EXIT /B 1
)

SET /A VehicleTotalNum=%VehicleNum% + %START_INDEX% - 1

TASKLIST|FIND /I "QGroundControl.exe" >NUL || START "" "%PSP_PATH%\QGroundControl\QGroundControl.exe" -noComPix

CD /D "%PSP_PATH%\RflySim3D"
TASKLIST|FIND /I "RflySim3D.exe" >NUL || START "" "%PSP_PATH%\RflySim3D\RflySim3D.exe"
CHOICE /T 5 /D Y /N >NUL

CD /D "%PSP_PATH%\CopterSim"
SET /A cntr=%START_INDEX%
SET string=%PosXStr%
SET stringY=%PosYStr%
SET stringYaw=%YawStr%
SET stringCom=%ComNum%

:START_COPTER_LOOP
FOR /F "tokens=1,* delims=," %%i IN ("!stringCom!") DO (
    SET comPort=%%i
    SET stringCom=%%j
)
FOR /F "tokens=1,* delims=," %%i IN ("!string!") DO (
    SET xPos=%%i
    SET string=%%j
)
FOR /F "tokens=1,* delims=," %%i IN ("!stringY!") DO (
    SET yPos=%%i
    SET stringY=%%j
)
FOR /F "tokens=1,* delims=," %%i IN ("!stringYaw!") DO (
    SET yawAng=%%i
    SET stringYaw=%%j
)

START /REALTIME CopterSim.exe 1 !cntr! %CLASS_3D_ID% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% !xPos! !yPos! !yawAng! !comPort!:%BaudRate% %UDPSIMMODE% %IsSysID% %IsAltAng% %isPosGps%,%isBatLLAOrin%,%LatLongAlt%
CHOICE /T 3 /D Y /N >NUL
SET /A cntr=!cntr!+1

IF !cntr! GTR %VehicleTotalNum% GOTO START_CONTROLLER
IF NOT "!stringCom!"=="" GOTO START_COPTER_LOOP

:START_CONTROLLER
ECHO Wait until all six CopterSim windows show PX4 GPS/EKF ready.
PAUSE

START "Six UAV BTS Controller" CMD /K "%~dp0RUN_CONTROLLER_EXAMPLE.bat"

ENDLOCAL
