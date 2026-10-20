#include <windows.h>
#include <process.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
int wmain(int argc, wchar_t** argv) {
    namespace fs = std::filesystem;
    wchar_t module[32768];
    GetModuleFileNameW(nullptr,module,32768);
    const auto bin=fs::path(module).parent_path();
    auto root=bin;
    if(!fs::exists(root/L"scripts/run_team_mvp.py"))root=fs::path(C2_SOURCE_ROOT);
    const auto script=root/L"scripts/run_team_mvp.py";
    auto python=root/L".venv-team/Scripts/python.exe";
    if(const auto* configured=_wgetenv(L"C2_PYTHON"))python=configured;
    else if(!fs::exists(python))python=L"python";
    std::vector<std::wstring> arguments{python.wstring(),script.wstring(),L"--detector",(bin/L"c2_detector_worker.exe").wstring()};
    for(int i=1;i<argc;++i)arguments.emplace_back(argv[i]);
    std::vector<const wchar_t*> pointers;
    for(const auto& argument:arguments)pointers.push_back(argument.c_str());
    pointers.push_back(nullptr);
    const auto result=_wspawnvp(_P_WAIT,python.c_str(),pointers.data());
    if(result==-1){std::wcerr<<L"Cannot start Python. Run scripts/setup-team.ps1 or set C2_PYTHON.\n";return 1;}
    return static_cast<int>(result);
}
