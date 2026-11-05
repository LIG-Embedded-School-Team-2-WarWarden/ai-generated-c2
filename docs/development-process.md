# 빌드와 검증

현재 대상은 Visual Studio 기반 C2.Console/Core/Ipc/Tests입니다.
Windows C++ 도구로 빌드하며 nlohmann JSON 헤더·라이선스는 C2.Core/Include/nlohmann에 있습니다.
Python 3.13은 모의기와 통합 시험에 사용합니다. WPF는 별도 GUI 저장소의 실행 파일입니다.

## 로컬 실행

프로젝트 루트에서 실행합니다.

```powershell
.\scripts\build.ps1 -Configuration Release
.\scripts\test.ps1 -Configuration Release
```

PowerShell 정책으로 ps1이 차단되면 Visual Studio에서 Release/x64 솔루션을 빌드하고 직접 시험합니다.

```powershell
.\x64\Release\C2.Tests.exe
python .\C2.Tests\Integration\test_station.py --console .\x64\Release\C2.Console.exe
python .\C2.Tests\Integration\test_simulator.py --console .\x64\Release\C2.Console.exe
```

| 시험 | 확인 사항 |
|---|---|
| C2.Tests.exe | RDR 제어 24바이트·DE03 32바이트·FCS 8/12바이트, 장비 fixture, 응답 검증·세션 |
| test_station.py | UDP·Windows IPC v3, 소수 각도, XYZ, 구동·정지, CLI·종료 |
| test_simulator.py | 표적10개, 할당·데모·취소, 위치에 따른 FCS 각도·정렬 무효화, 소실·결함·단절·복구 |

자동 시험은 별도 IPC 이름으로 기본 통제소와 분리합니다.
원본 WPF를 포함한 모의기는 기본 IPC를 사용하므로 기존 C2와 동시에 켜지 않습니다.
통신 시험은 실제 WPF 화면이나 실물 센서·모터·출력을 검증하지 않습니다.
화면 기록과 제한은 [WPF 검증](wpf-verification.md)에 있습니다.

CI는 `.github/workflows/build.yml`에서 Windows Release 빌드와 같은 시험을 실행합니다.
구형 Linux/CMake/Protobuf 시험은 현재 CI에 포함되지 않습니다.
