#include "c2/diagnostic_log.hpp"
#include <vector>
#include <stdexcept>

int diagnostic_log_entry() {
    const auto dir=std::filesystem::temp_directory_path()/
        ("c2-trace-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);
    try {
        {
            c2::DiagnosticLog logger;
            logger.start(dir/"trace.jsonl","rotation-test",2048);
            std::vector<std::thread> producers;
            for(int t=0;t<4;++t)producers.emplace_back([&,t]{
                for(int i=0;i<200;++i)logger.emit("test","sample",{{"thread",t},{"index",i}});
            });
            for(auto& producer:producers)producer.join();
        } // Flush and join before inspecting files.
        int files=0,rows=0;
        for(const auto& entry:std::filesystem::directory_iterator(dir)) {
            ++files;
            if(entry.file_size()>2048)throw std::runtime_error("trace rotation size exceeded");
            std::ifstream input(entry.path());std::string line;
            while(std::getline(input,line)) {
                const auto record=nlohmann::json::parse(line);
                if(record.at("run")!="rotation-test"||record.at("dropped_total")!=0)
                    throw std::runtime_error("trace corrupt or dropped");
                ++rows;
            }
        }
        if(files!=4||rows==0)throw std::runtime_error("trace rotation/flush failed");
    } catch(...) {std::filesystem::remove_all(dir);throw;}
    std::filesystem::remove_all(dir);
    return 0;
}
