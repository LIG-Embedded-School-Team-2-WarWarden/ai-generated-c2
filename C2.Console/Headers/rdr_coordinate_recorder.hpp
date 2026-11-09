#pragma once
#include "rdr_coordinates.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <tuple>

namespace c2::rdr {
// Optional read-only trace of the exact input/output snapshot published to GUI.
class CoordinateRecorder {
    std::filesystem::path path_,metadata_path_;
    std::ofstream output_;
    nlohmann::json metadata_;
    double duration_,started_{},last_now_{};
    uint64_t last_unix_ms_{};
    bool started_flag_{},finished_{};
    size_t rows_{};
    using Key=std::tuple<uint64_t,double,uint16_t,uint8_t,bool,double,double,double>;
    std::map<uint16_t,Key> recorded_;
    void metadata(const char* status) {
        metadata_["status"]=status;metadata_["rows"]=rows_;
        metadata_["updated_unix_ms"]=last_unix_ms_;
        metadata_["elapsed_s"]=started_flag_?last_now_-started_:0.;
        std::ofstream out(metadata_path_);out<<metadata_.dump(2)<<'\n';
    }
    void finish(const char* status) {
        output_.flush();output_.close();finished_=true;metadata(status);
    }
public:
    CoordinateRecorder(std::filesystem::path path,double seconds,nlohmann::json context)
        : path_(std::filesystem::absolute(path)),metadata_path_(path_.string()+".json"),
          metadata_(std::move(context)),duration_(seconds) {
        if(!std::isfinite(duration_) || duration_<=0 || duration_>3600)
            throw std::invalid_argument("rdr-record-seconds must be greater than 0 and at most 3600");
        if(std::filesystem::exists(path_) || std::filesystem::exists(metadata_path_))
            throw std::invalid_argument("coordinate recording output already exists");
        std::filesystem::create_directories(path_.parent_path());
        output_.open(path_);if(!output_)throw std::runtime_error("cannot create coordinate recording");
        output_<<std::setprecision(17);
        output_<<"capture_unix_ms,elapsed_s,generation,target_id,received_s,track_state,detected,status,valid,"
            "sensor_x,sensor_y,sensor_z,observation_pan,observation_tilt,origin_x,origin_y,origin_z,"
            "before_x,before_y,before_z,after_x,after_y,after_z\n";
        output_.flush();
        metadata_["schema_version"]=1;metadata_["requested_duration_s"]=duration_;
        metadata_["csv_path"]=path_.string();metadata_["coordinate_frame"]="world east,north,up metres";
        metadata_["sampling"]="new received samples consumed by GUI publisher; identical receive timestamps are deduplicated";
        metadata("waiting_for_valid_target");
    }
    ~CoordinateRecorder() {if(!finished_) {try{finish("interrupted");}catch(...) {}}}
    bool finished() const {return finished_;}
    void append(const State& state,BasePoint origin,const std::vector<CorrectedTarget>& displayed,
                bool operating,double now,uint64_t unix_ms) {
        if(finished_)return;
        last_now_=now;last_unix_ms_=unix_ms;
        if(!started_flag_) {
            if(!operating || std::none_of(displayed.begin(),displayed.end(),[](const auto& p){return p.valid;}))return;
            started_flag_=true;started_=now;metadata_["started_unix_ms"]=unix_ms;metadata("recording");
        }
        if(now-started_>=duration_) {finish("completed");return;}
        const auto before=correct_targets(state,origin);
        for(const auto& input:before) {
            const auto& raw=state.targets.at(input.id);
            const auto after=std::find_if(displayed.begin(),displayed.end(),[&](const auto& p){return p.id==input.id;});
            const bool valid=operating && input.valid && after!=displayed.end() && after->valid;
            const Key key{state.generation,raw.seen,raw.track_state,raw.status,valid,origin.east,origin.north,origin.up};
            const auto previous=recorded_.find(input.id);
            if(previous!=recorded_.end() && previous->second==key)continue;
            recorded_.insert_or_assign(input.id,key);
            output_<<unix_ms<<','<<now-started_<<','<<state.generation<<','<<input.id<<','<<raw.seen<<','
                <<raw.track_state<<','<<raw.detected<<','<<unsigned(raw.status)<<','<<valid<<','
                <<raw.x<<','<<raw.y<<','<<raw.z<<','<<raw.pan<<','<<raw.tilt<<','
                <<origin.east<<','<<origin.north<<','<<origin.up;
            if(valid) output_<<','<<input.world.east<<','<<input.world.north<<','<<input.world.up
                <<','<<float(after->world.east)<<','<<float(after->world.north)<<','<<float(after->world.up);
            else output_<<",,,,,,";
            output_<<'\n';++rows_;
        }
        output_.flush();
        if(!output_) {metadata_["error"]="coordinate CSV write failed";finish("failed");}
    }
};
} // namespace c2::rdr
