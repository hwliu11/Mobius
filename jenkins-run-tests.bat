@echo off

echo "*** Running jenkins-run-tests.bat..."

REM ===========================================================================
REM Configure environment on Jenkins machine
REM ===========================================================================

call "%~dp0"jenkins-custom.bat

set "JENKINS_3RDPARTIES=%~dp03rd-parties"

set "MOBIUS_TEST_DUMPING=%~dp0cmake-build-dir"
set "MOBIUS_TEST_DATA=%~dp0data"
set "MOBIUS_TEST_DESCR=%~dp0src/test"
rem set "ASI_TEST_DESCR=@ASI_TEST_DESCR@"

setlocal enabledelayedexpansion

set "MOBIUS_TEST_DUMPING=!MOBIUS_TEST_DATA:\=/!"
echo "MOBIUS_TEST_DUMPING=!MOBIUS_TEST_DUMPING!"

set "MOBIUS_TEST_DATA=!MOBIUS_TEST_DATA:\=/!"
echo "MOBIUS_TEST_DATA=!MOBIUS_TEST_DATA!"

set "MOBIUS_TEST_DESCR=!MOBIUS_TEST_DESCR:\=/!"
echo "MOBIUS_TEST_DESCR=!MOBIUS_TEST_DESCR!"

REM ===========================================================================
REM Run tests
REM ===========================================================================

cd cmake-install-dir/bin

mobiusTest.exe

REM ===========================================================================
REM Copy test results to the network drive
REM ===========================================================================

rem The following lines are commented out as there seems to be no sense in
rem copying test result to file server: all them are accessible via http
rem in Jenkins workspace, e.g. http://ssv:8080/.../...

rem echo Test results will be available at "%JENKINS_TEST_RESULT_DIR%"
rem cd %JENKINS_JOB_DIR%\test\results
rem xcopy /s . %JENKINS_TEST_RESULT_DIR% > NUL
