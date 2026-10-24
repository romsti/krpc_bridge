@echo off
rem ---------------------------------------------------------------------------
rem  KRPC.Bridge build helper (Windows)
rem
rem    build.cmd                            find KSP by itself, build to dist\
rem    build.cmd deploy                     ... and copy into GameData
rem    build.cmd "D:\Games\KSP_1.12.5"      use that install, and remember it
rem    build.cmd "D:\Games\KSP_1.12.5" deploy
rem    build.cmd verify                     what CI runs: type-check, tests, docs - no KSP
rem    build.cmd forget                     drop the remembered KSP path
rem
rem  Every DLL is stamped with the commit it was built from (tools\build_stamp.py), and the
rem  last step writes dist\GameData\KRPC.Bridge\manifest.json and dist\GameData\TowerCatch\manifest.json
rem  (tools\manifest.py): sha256 of every file, the commit, clean or dirty. "deploy" below is the
rem  plain copy into one install.
rem
rem  Two mod folders come out: KRPC.Bridge (the DLLs) and TowerCatch (the catch tower's ModuleManager
rem  patch and craft, from distribution\TowerCatch; no DLL - its services stay in KRPC.Bridge.Tower.dll).
rem
rem  YOU DO NOT NORMALLY NEED TO GIVE THE PATH. tools\find_ksp.py looks at the KSPROOT
rem  variable, at ksp.path in this folder, at every Steam library declared in
rem  libraryfolders.vdf - which is what finds an install on a second drive - and then at
rem  the usual locations. Give the path once if all that fails and it is remembered.
rem
rem  A path is needed at all only because the build compiles against the game's own
rem  assemblies, which may not be redistributed and so cannot live in this repo.
rem
rem  Output lands in dist\GameData\, laid out exactly like the destination.
rem  Nothing is written into your install unless you pass "deploy".
rem ---------------------------------------------------------------------------
setlocal EnableDelayedExpansion

set "REPO=%~dp0"
set "DIST=%REPO%dist\GameData\KRPC.Bridge"
set "DIST_TOWER=%REPO%dist\GameData\TowerCatch"

if /I "%~1"=="verify" goto :verify
if /I "%~1"=="forget" (
  python "%REPO%tools\find_ksp.py" --forget
  exit /b 0
)

rem  First argument is either a KSP path or the word "deploy".
set "KSP=%~1"
set "DEPLOY=%~2"
if /I "%KSP%"=="deploy" (
  set "KSP="
  set "DEPLOY=deploy"
)

echo.
echo === 1/4  type-check against stubs (no KSP needed) ===
dotnet build "%REPO%build\verify\Verify.csproj" -v minimal --nologo
if errorlevel 1 goto :failed

echo.
echo === 1b/4 AttitudeV1 maths + reflection-cache tests (no KSP needed) ===
rem  WLS against a brute-force QP, reference/hedge/filter properties, QW6 readers against
rem  the legacy ones; rewrites build\tests\golden\attitude_v1_golden.json (deterministic).
dotnet run --project "%REPO%build\tests\AttitudeTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1c/4 FMRS dropped-stage table tests (no KSP needed) ===
rem  DroppedStagesV1: the ledger that binds each FMRS entry to its staging and separation,
rem  replayed event by event (a first campaign flight, a Dres lander, a revert), and its wire format.
dotnet run --project "%REPO%build\tests\fmrs\FmrsTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1d/4 manifest tests (no KSP needed) ===
rem  conn.bridge.manifest_v1: build stamp read back, services, layout constants, header verdict.
dotnet run --project "%REPO%build\tests\manifest\ManifestTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1e/4 Core v2 tests: recorder, lease, clock, bits, frames (no KSP needed) ===
rem  The frame Recorder against a verbatim copy of the LinkedList rings it replaced, the Lease
rem  against the legacy float arithmetic, bit for bit.
dotnet run --project "%REPO%build\tests\core\CoreTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1f/4 Tower hinge lock at rest tests (no KSP needed) ===
rem  HingeRestWatch (conn.tower.hinge_lock_at_rest_v1): the turning point of a vibration found
rem  on the physics step (a recorded flight), the time limit, the order of the tests; its layout declared
rem  into the real LayoutRegistry.
dotnet run --project "%REPO%build\tests\tower\TowerTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1g/4 ContactV1 latch tests (no KSP needed) ===
rem  ContactLatch (conn.telemetry.contact_v1, refactor lot 3): touchdown, every leg, all legs, Landed,
rem  the LANDED situation, a bounce, a body contact, a collision alone, a rescan, no allocation per step.
dotnet run --project "%REPO%build\tests\contact\ContactTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 1h/4 staging lock tests (no KSP needed) ===
rem  StagingLocks (conn.stock.stage_in_frame_v2, staging_locks_v1): which input locks hold the
rem  staging, and that the focus lock KSP sets while its window is in the background stops no
rem  staging asked by a program, while the Alt+L lock and every other one still do.
dotnet run --project "%REPO%build\tests\stock\StockTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed

echo.
echo === 2/4  locating KSP ===
if not "%KSP%"=="" (
  rem  An explicit path wins, and is remembered so this is the last time you type it.
  if not exist "%KSP%\KSP_x64_Data\Managed\Assembly-CSharp.dll" (
    echo   NOT A KSP INSTALL: "%KSP%"
    echo   Expected "%KSP%\KSP_x64_Data\Managed\Assembly-CSharp.dll".
    goto :failed
  )
  > "%REPO%ksp.path" echo %KSP%
  echo   %KSP%   ^(given, and remembered for next time^)
) else (
  for /f "usebackq delims=" %%K in (`python "%REPO%tools\find_ksp.py" --quiet 2^>nul`) do set "KSP=%%K"
  if "!KSP!"=="" (
    python "%REPO%tools\find_ksp.py"
    goto :failed
  )
  echo   !KSP!   ^(found automatically^)
  set "KSP=!KSP!"
)

if not exist "!KSP!\GameData\kRPC\KRPC.Core.dll" (
  echo.
  echo   kRPC IS NOT INSTALLED in that copy of KSP.
  echo   Expected "!KSP!\GameData\kRPC\KRPC.Core.dll".
  echo   Contents of GameData\kRPC:
  dir /b "!KSP!\GameData\kRPC\*.dll" 2>nul
  echo.
  echo   Install kRPC 0.7.0 there, or point at another copy:
  echo       .\build.cmd "path\to\other\KSP"
  goto :failed
)
set "KSP=!KSP!"

echo.
echo === 3/4  build the solution ===
rem  The stamp is taken ONCE, here: after the tests (which rewrite the golden file, so a stale
rem  golden shows as a dirty tree) and before the compile, and every project gets the same file.
python "%REPO%tools\build_stamp.py" --write "%REPO%obj"
if errorlevel 1 goto :failed
rem  dist\ starts empty: a DLL that a refactor renamed or retired (R3: Actuators, StockDeltaV)
rem  would otherwise linger there, be scanned as a second commit and, worse, be deployed next to
rem  its successor. dist\ holds nothing but build output.
if exist "%DIST%" rmdir /s /q "%DIST%"
dotnet build "%REPO%KRPC.Bridge.sln" -c Release -p:KSPRoot="%KSP%" -p:BridgeStampFile="%REPO%obj\BridgeStamp.g.cs" -v minimal --nologo
if errorlevel 1 goto :failed

rem  The licence, the notice and the KSP-AVC version file, so dist\GameData\KRPC.Bridge is the whole mod folder.
copy /Y "%REPO%LICENSE" "%DIST%\LICENSE" >nul
if errorlevel 1 goto :failed
copy /Y "%REPO%NOTICE" "%DIST%\NOTICE" >nul
if errorlevel 1 goto :failed
copy /Y "%REPO%distribution\KRPC.Bridge.version" "%DIST%\KRPC.Bridge.version" >nul
if errorlevel 1 goto :failed

rem  The catch tower is a package of its own, dist\GameData\TowerCatch: the ModuleManager patch that makes its
rem  parts (TG.*), the craft those parts build (conn.tower_site spawns it when the game has no copy), its notice
rem  and its KSP-AVC version file (all of distribution\TowerCatch), plus the licence. They used to be copied
rem  into KRPC.Bridge; the bridge no longer ships them (two copies of the patch make every part twice).
rem  dist\ starts empty here too, so a file the package dropped does not linger.
if exist "%DIST_TOWER%" rmdir /s /q "%DIST_TOWER%"
xcopy /E /I /Y /Q "%REPO%distribution\TowerCatch" "%DIST_TOWER%\" >nul
if errorlevel 1 goto :failed
copy /Y "%REPO%LICENSE" "%DIST_TOWER%\LICENSE" >nul
if errorlevel 1 goto :failed

echo.
echo === 4/4  validate every kRPC signature with kRPC's own scanner ===
rem  This is the check that matters. One malformed signature in ANY loaded assembly
rem  disables the whole kRPC server in game, not just the offending service - so
rem  catching it here, in a second, is worth more than every other test in the repo.
rem  NO --nologo on this line. It is not an option `dotnet run` understands, so it gets
rem  forwarded to the tool, which reads it as the KSP path and fails with
rem  "KSP managed assemblies not found under '--nologo'".
rem
rem  The scanner is given every DLL found in dist\, which is also what deploy copies. A DLL
rem  that never landed there would simply not be scanned, and nothing would say so - so
rem  every assembly that ships is named here and must be present first. Keep this list in
rem  step with KRPC.Bridge.sln and with EXPECTED in tools\package.py.
set "MISSING="
for %%N in (
  KRPC.Bridge.Core.dll
  Plugins\KRPC.Bridge.Model.dll
  Plugins\KRPC.Bridge.Control.dll
  Plugins\KRPC.Bridge.Telemetry.dll
  Plugins\KRPC.Bridge.Fmrs.dll
  Plugins\KRPC.Bridge.Ident.dll
  Plugins\KRPC.Bridge.MechJeb.dll
  Plugins\KRPC.Bridge.Ocisly.dll
  Plugins\KRPC.Bridge.Stock.dll
  Plugins\KRPC.Bridge.Tower.dll
  Plugins\KRPC.Bridge.Trajectories.dll
) do (
  if not exist "%DIST%\%%N" (
    echo   MISSING FROM dist: %%N
    set "MISSING=1"
  )
)
if defined MISSING goto :failed
set "SCANARGS="
for %%F in ("%DIST%\*.dll" "%DIST%\Plugins\*.dll") do set "SCANARGS=!SCANARGS! "%%~fF""
dotnet run --project "%REPO%build\scan\Scan.csproj" -c Release -v quiet -- "!KSP!" !SCANARGS!
if errorlevel 1 (
  echo.
  echo   SIGNATURE OR MANIFEST CHECK FAILED - do NOT deploy this build.
  echo   A bad signature would disable the entire kRPC server in game.
  goto :failed
)

echo.
echo === manifest: sha256 of every shipped file of each package, the commit, clean or dirty ===
python "%REPO%tools\manifest.py" --package KRPC.Bridge --stamp "%REPO%obj\BridgeStamp.json" --dist "%DIST%"
if errorlevel 1 goto :failed
python "%REPO%tools\manifest.py" --package TowerCatch --stamp "%REPO%obj\BridgeStamp.json" --dist "%DIST_TOWER%"
if errorlevel 1 goto :failed

if /I not "!DEPLOY!"=="deploy" goto :done

echo.
echo === deploying to GameData ===
rem  The tower's patch moved out of KRPC.Bridge into the TowerCatch package. A plain copy never removes a file, so
rem  copying both over an install that still has the old patch would leave TWO copies of it (every TG.* part made
rem  twice). Refuse before writing anything.
if exist "%KSP%\GameData\KRPC.Bridge\Patches\TowerCatch.cfg" (
  echo   REFUSED: "%KSP%\GameData\KRPC.Bridge\Patches\TowerCatch.cfg" is the tower patch of the bridge before the
  echo   TowerCatch package. Installing the package next to it makes two copies of the patch.
  echo   Delete GameData\KRPC.Bridge\Patches and GameData\KRPC.Bridge\Crafts by hand, KSP closed, and run this again.
  goto :failed
)
xcopy /E /I /Y /Q "%REPO%dist\GameData\KRPC.Bridge" "%KSP%\GameData\KRPC.Bridge\" >nul
if errorlevel 1 goto :failed
echo   copied to "%KSP%\GameData\KRPC.Bridge\"
xcopy /E /I /Y /Q "%DIST_TOWER%" "%KSP%\GameData\TowerCatch\" >nul
if errorlevel 1 goto :failed
echo   copied to "%KSP%\GameData\TowerCatch\"
echo.
echo   If you are upgrading from the single-DLL version, DELETE the old
echo   GameData\KRPC.Bridge\Plugins\KRPC.Bridge.dll by hand. Leaving it there makes
echo   two assemblies declare the same kRPC service names, and kRPC refuses the
echo   duplicate - which takes the whole server down.
goto :after

:done
echo.
echo Built to %REPO%dist\GameData\KRPC.Bridge\ and %REPO%dist\GameData\TowerCatch\
echo Copy those folders into "!KSP!\GameData\", or re-run with:
echo    .\build.cmd deploy

:after
echo.
echo Then start KSP, start the kRPC server, and run:
echo    python "%REPO%python\check_bridge.py"
exit /b 0

:verify
rem  The same steps as CI (.github\workflows\build.yml), in the same order. Keep them in step.
echo.
echo === 1/5  type-check against stubs (no KSP needed) ===
dotnet build "%REPO%build\verify\Verify.csproj" -v minimal --nologo
if errorlevel 1 goto :failed
echo.
echo === 2/5  tests without KSP: AttitudeV1, FMRS table, manifest, Core v2, Tower, ContactV1, staging locks ===
dotnet run --project "%REPO%build\tests\AttitudeTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\fmrs\FmrsTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\manifest\ManifestTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\core\CoreTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\tower\TowerTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\contact\ContactTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
dotnet run --project "%REPO%build\tests\stock\StockTests.csproj" -c Release -v quiet
if errorlevel 1 goto :failed
echo.
echo === 3/5  the signature scanner compiles ===
dotnet build "%REPO%build\scan\Scan.csproj" -v minimal --nologo
if errorlevel 1 goto :failed
echo.
echo === 4/5  build tools: stamp, manifest, package (unit tests) ===
python -m unittest discover -s "%REPO%tools" -p "test_*.py"
if errorlevel 1 goto :failed
echo.
echo === 5/5  docs match the code ===
python "%REPO%tools\check_docs.py"
if errorlevel 1 goto :failed
echo.
echo Verified. This produces NOTHING usable in game - the type-check links the stubs in
echo build\stubs\, and it does NOT validate kRPC signatures. Run build.cmd with your KSP
echo path for the real DLLs, the signature scan and the manifest.
exit /b 0

:failed
echo.
echo BUILD FAILED
exit /b 1
