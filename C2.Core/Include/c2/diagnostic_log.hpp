#pragma once
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

namespace c2 {
// Bounded asynchronous diagnostics: disk I/O never runs on device callbacks.
class DiagnosticLog {
    std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<std::string> queue_;
    std::thread worker_;
    std::atomic_bool enabled_{};
    bool stopping_{};
    std::atomic_uint64_t dropped_{};
    std::filesystem::path path_;
    std::ofstream file_;
    uint64_t bytes_{}, limit_{64*1024*1024};
    std::string run_;
    std::chrono::steady_clock::time_point start_;
    void write(const std::string& line) {
        if(bytes_+line.size()+1>limit_) {
            file_.close();
            auto backup=[&](int n){return std::filesystem::path(path_.string()+"."+std::to_string(n));};
            std::filesystem::remove(backup(3));
            for(int n=2;n>=1;--n)if(std::filesystem::exists(backup(n)))
                std::filesystem::rename(backup(n),backup(n+1));
            std::filesystem::rename(path_,backup(1));
            file_.open(path_,std::ios::trunc);bytes_=0;
        }
        file_<<line<<'\n';bytes_+=line.size()+1;
        if(!file_)throw std::runtime_error("diagnostic write failed");
    }
    void loop() noexcept {
        try {
            for(;;) {
                std::deque<std::string> batch;
                {std::unique_lock lock(mutex_);
                 wake_.wait_for(lock,std::chrono::milliseconds(250),[&]{return stopping_||!queue_.empty();});
                 if(stopping_&&queue_.empty())break;
                 batch.swap(queue_);}
                for(const auto& line:batch)write(line);
                file_.flush();
                if(!file_)throw std::runtime_error("diagnostic flush failed");
            }
        } catch(const std::exception& e) {
            enabled_=false;
            std::cerr<<"Device diagnostics disabled: "<<e.what()<<" path="<<path_.string()<<std::endl;
        }
    }
public:
    static DiagnosticLog& instance(){static DiagnosticLog log;return log;}
    ~DiagnosticLog(){enabled_=false;{std::lock_guard lock(mutex_);stopping_=true;}wake_.notify_one();if(worker_.joinable())worker_.join();}
    void start(const std::filesystem::path& path,std::string run,uint64_t limit=64*1024*1024) {
        if(worker_.joinable())throw std::logic_error("diagnostics already started");
        path_=path;run_=std::move(run);limit_=limit;start_=std::chrono::steady_clock::now();
        file_.open(path_,std::ios::app);
        if(!file_)throw std::runtime_error("cannot open device diagnostics: "+path_.string());
        bytes_=std::filesystem::file_size(path_);enabled_=true;
        worker_=std::thread([this]{loop();});
    }
    void emit(const char* source,const char* event,nlohmann::json fields) noexcept {
        if(!enabled_)return;
        try {
            fields["schema"]=1;fields["run"]=run_;fields["source"]=source;fields["event"]=event;
            fields["unix_us"]=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            fields["mono_us"]=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start_).count();
            fields["dropped_total"]=dropped_.load();
            auto line=fields.dump();
            {std::lock_guard lock(mutex_);if(queue_.size()>=8192){++dropped_;return;}queue_.push_back(std::move(line));}
            wake_.notify_one();
        }catch(...){++dropped_;}
    }
};
inline void diagnostic(const char* source,const char* event,nlohmann::json fields) noexcept {
    DiagnosticLog::instance().emit(source,event,std::move(fields));
}
}
