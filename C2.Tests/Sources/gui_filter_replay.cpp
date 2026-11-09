#include "target_position_filter.hpp"
#include <fstream>
#include <iostream>

// Offline calibration uses the production filter, with no sockets or devices.
int gui_filter_replay_entry(const char* input_path,const char* output_path) {
    std::ifstream input(input_path);
    if(!input)throw std::runtime_error("replay input missing");
    nlohmann::json request;input>>request;
    nlohmann::json runs=nlohmann::json::array();
    for(const auto& config:request.at("configs")) {
        for(const auto& dataset:request.at("datasets")) {
            c2::rdr::TargetPositionFilter filter(c2::rdr::parse_target_position_filter(config.at("filter")));
            c2::rdr::State state;state.connected=state.reporting=true;state.mode=2;
            nlohmann::json values=nlohmann::json::array();
            for(const auto& row:dataset.at("samples")) {
                const auto generation=row.value("generation",uint64_t{1});
                if(state.generation!=generation)state.targets.clear();
                state.generation=generation;
                const auto id=row.at("id").get<uint16_t>();
                auto& target=state.targets[id];target.id=id;target.seen=row.at("seen");
                target.track_state=row.value("track_state",uint16_t{1});
                target.detected=target.track_state==1;target.status=target.track_state && row.value("valid",true)?0:2;
                c2::rdr::BasePoint origin;
                if(row.contains("origin"))origin={row.at("origin").at(0),row.at("origin").at(1),row.at("origin").at(2)};
                // Input is already in world axes; a neutral pose preserves it.
                target.pan=1800;target.tilt=0;
                target.x=-(row.at("world").at(0).get<double>()-origin.east);
                target.z=row.at("world").at(1).get<double>()-origin.north;
                target.y=row.at("world").at(2).get<double>()-origin.up;
                const auto points=filter.update(state,origin);
                const auto point=std::find_if(points.begin(),points.end(),[&](const auto& p){return p.id==id;});
                values.push_back(point->valid?nlohmann::json::array({float(point->world.east),
                    float(point->world.north),float(point->world.up)}):nlohmann::json(nullptr));
            }
            runs.push_back({{"config",config.at("name")},{"dataset",dataset.at("name")},{"xyz",values}});
        }
    }
    std::ofstream output(output_path);
    if(!output)throw std::runtime_error("cannot create replay output");
    output<<nlohmann::json{{"runs",runs}}.dump()<<'\n';
    if(!output)throw std::runtime_error("cannot write replay output");
    std::cout<<"PASS: offline production-filter replay, no device connections\n";
    return 0;
}
