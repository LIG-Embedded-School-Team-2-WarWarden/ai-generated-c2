#pragma once
#include "rdr_coordinates.hpp"
#include <array>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace c2::rdr {
struct TargetPositionFilterConfig {
    bool enabled{true};
    double measurement_stddev_m{.05};
    double process_stddev_m_sqrt_s{.10};
    bool adaptive_motion{false};
    double motion_process_stddev_m_sqrt_s{.08};
    double motion_trigger_sigma{2.5};
    // Optional world-axis overrides, ordered east (X), north (Y), up (Z).
    std::optional<std::array<double,3>> measurement_stddev_xyz_m;
    std::optional<std::array<double,3>> process_stddev_xyz_m_sqrt_s;
    std::optional<std::array<double,3>> motion_process_stddev_xyz_m_sqrt_s;
};

inline std::array<double,3> gui_noise_axes(const std::optional<std::array<double,3>>& axes,double scalar) {
    return axes.value_or(std::array<double,3>{scalar,scalar,scalar});
}

inline TargetPositionFilterConfig parse_target_position_filter(const nlohmann::json& json) {
    if (!json.is_object()) throw std::invalid_argument("rdr.gui_kalman must be an object");
    TargetPositionFilterConfig config;
    if (json.contains("enabled")) config.enabled=json.at("enabled").get<bool>();
    if (json.contains("adaptive_motion")) config.adaptive_motion=json.at("adaptive_motion").get<bool>();
    auto read=[&](const char* key, double& value) {
        if (!json.contains(key)) return;
        if (!json.at(key).is_number()) throw std::invalid_argument(std::string("rdr.gui_kalman.")+key+" must be numeric");
        value=json.at(key).get<double>();
    };
    read("measurement_stddev_m",config.measurement_stddev_m);
    read("process_stddev_m_sqrt_s",config.process_stddev_m_sqrt_s);
    read("motion_process_stddev_m_sqrt_s",config.motion_process_stddev_m_sqrt_s);
    read("motion_trigger_sigma",config.motion_trigger_sigma);
    auto read_axes=[&](const char* key,std::optional<std::array<double,3>>& values) {
        if (!json.contains(key))return;
        const auto& array=json.at(key);
        if(!array.is_array() || array.size()!=3 ||
            !std::all_of(array.begin(),array.end(),[](const auto& value){return value.is_number();}))
            throw std::invalid_argument(std::string("rdr.gui_kalman.")+key+" must contain three numbers in XYZ order");
        values=array.get<std::array<double,3>>();
    };
    read_axes("measurement_stddev_xyz_m",config.measurement_stddev_xyz_m);
    read_axes("process_stddev_xyz_m_sqrt_s",config.process_stddev_xyz_m_sqrt_s);
    read_axes("motion_process_stddev_xyz_m_sqrt_s",config.motion_process_stddev_xyz_m_sqrt_s);
    return config;
}

inline nlohmann::json target_position_filter_json(const TargetPositionFilterConfig& config) {
    return {{"model","position_random_walk"},{"enabled",config.enabled},
        {"measurement_stddev_m",config.measurement_stddev_m},
        {"process_stddev_m_sqrt_s",config.process_stddev_m_sqrt_s},
        {"adaptive_motion",config.adaptive_motion},
        {"motion_process_stddev_m_sqrt_s",config.motion_process_stddev_m_sqrt_s},
        {"motion_trigger_sigma",config.motion_trigger_sigma},
        {"measurement_stddev_xyz_m",gui_noise_axes(config.measurement_stddev_xyz_m,config.measurement_stddev_m)},
        {"process_stddev_xyz_m_sqrt_s",gui_noise_axes(config.process_stddev_xyz_m_sqrt_s,config.process_stddev_m_sqrt_s)},
        {"motion_process_stddev_xyz_m_sqrt_s",gui_noise_axes(config.motion_process_stddev_xyz_m_sqrt_s,config.motion_process_stddev_m_sqrt_s)}};
}

// Shared target-position Kalman filter: independent world XYZ with diagonal
// Q/R. Sender predictions receive conservative smoothing, without an
// independent-measurement covariance reduction. Received data stays unchanged.
class TargetPositionFilter {
    struct Estimate {
        BasePoint world; std::array<double,3> variance{}; double seen{};
        std::optional<BasePoint> motion_candidate;
        double candidate_seen{};
        unsigned motion_updates_left{};
    };
    TargetPositionFilterConfig config_;
    std::array<double,3> measurement_variance_{}, process_variance_rate_{},motion_variance_rate_{};
    std::map<uint16_t,Estimate> estimates_;
    uint64_t generation_{};
    BasePoint origin_{};
    static constexpr double reset_gap_s=3.;
public:
    explicit TargetPositionFilter(TargetPositionFilterConfig config={}) : config_(config) {
        const auto measurement=gui_noise_axes(config.measurement_stddev_xyz_m,config.measurement_stddev_m);
        const auto process=gui_noise_axes(config.process_stddev_xyz_m_sqrt_s,config.process_stddev_m_sqrt_s);
        const auto motion=gui_noise_axes(config.motion_process_stddev_xyz_m_sqrt_s,config.motion_process_stddev_m_sqrt_s);
        for(size_t axis=0;axis<3;++axis) {
            measurement_variance_[axis]=measurement[axis]*measurement[axis];
            process_variance_rate_[axis]=process[axis]*process[axis];
            motion_variance_rate_[axis]=motion[axis]*motion[axis];
            for(auto value:{measurement[axis],process[axis],motion[axis],measurement_variance_[axis],
                process_variance_rate_[axis],motion_variance_rate_[axis],config.motion_trigger_sigma})
                if(!std::isfinite(value) || value<=0)
                    throw std::invalid_argument("rdr.gui_kalman noise standard deviations must be finite, positive and squareable");
            const double max_prior=measurement_variance_[axis]+
                std::max(process_variance_rate_[axis],motion_variance_rate_[axis])*reset_gap_s;
            if(!std::isfinite(max_prior+measurement_variance_[axis]) ||
                !std::isfinite(config.motion_trigger_sigma*std::sqrt(max_prior+measurement_variance_[axis])))
                throw std::invalid_argument("rdr.gui_kalman noise variance is too large");
            if(config.adaptive_motion && motion_variance_rate_[axis]<process_variance_rate_[axis])
                throw std::invalid_argument("motion process noise must not be smaller than stationary process noise on any axis");
        }
    }
    void reset() { estimates_.clear(); }
    std::vector<CorrectedTarget> update(const State& state, BasePoint origin) {
        auto result=correct_targets(state,origin);
        if (generation_!=state.generation || origin.east!=origin_.east ||
            origin.north!=origin_.north || origin.up!=origin_.up) reset();
        generation_=state.generation;origin_=origin;
        if (!config_.enabled || !state.connected || !state.reporting || state.mode!=2) {
            reset();return result;
        }
        for (auto it=estimates_.begin();it!=estimates_.end();)
            if (!state.targets.contains(it->first)) it=estimates_.erase(it);else ++it;
        for (auto& point:result) {
            const auto& raw=state.targets.at(point.id);
            if (!point.valid || !std::isfinite(raw.seen)) {
                estimates_.erase(point.id);continue;
            }
            const bool observed=raw.track_state==1 && raw.detected==1;
            auto [it,inserted]=estimates_.try_emplace(point.id,
                Estimate{point.world,measurement_variance_,raw.seen});
            auto& estimate=it->second;
            const double dt=raw.seen-estimate.seen;
            if (dt<0 || dt>reset_gap_s) {
                estimate={point.world,measurement_variance_,raw.seen};
            } else if (!inserted && dt>0) {
                std::array<double,3> prior;
                for(size_t axis=0;axis<3;++axis)prior[axis]=estimate.variance[axis]+process_variance_rate_[axis]*dt;
                if (config_.adaptive_motion && observed) {
                    const BasePoint residual{point.world.east-estimate.world.east,
                        point.world.north-estimate.world.north,point.world.up-estimate.world.up};
                    // Compare innovations in noise-normalized world axes, so
                    // a noisy horizontal axis cannot mask a precise vertical one.
                    const BasePoint normalized{residual.east/std::sqrt(prior[0]+measurement_variance_[0]),
                        residual.north/std::sqrt(prior[1]+measurement_variance_[1]),
                        residual.up/std::sqrt(prior[2]+measurement_variance_[2])};
                    const double length=std::hypot(normalized.east,normalized.north,normalized.up);
                    const double threshold=config_.motion_trigger_sigma;
                    bool confirmed=false;
                    if (estimate.motion_candidate && raw.seen-estimate.candidate_seen<=reset_gap_s && length>threshold*.5) {
                        const auto& previous=*estimate.motion_candidate;
                        // Two fresh OBSERVED innovations within 60 degrees of
                        // each other. A single spike or a redraw cannot boost Q.
                        confirmed=(normalized.east/length)*previous.east+(normalized.north/length)*previous.north+
                            (normalized.up/length)*previous.up > .5;
                    }
                    if (confirmed) estimate.motion_updates_left=3;
                    estimate.motion_candidate.reset();
                    if (!confirmed && length>threshold) {
                        estimate.motion_candidate=BasePoint{normalized.east/length,normalized.north/length,normalized.up/length};
                        estimate.candidate_seen=raw.seen;
                    }
                    if (estimate.motion_updates_left) {
                        for(size_t axis=0;axis<3;++axis)prior[axis]=estimate.variance[axis]+motion_variance_rate_[axis]*dt;
                        --estimate.motion_updates_left;
                    }
                }
                const std::array<double,3> measured{point.world.east,point.world.north,point.world.up};
                const std::array<double*,3> filtered{&estimate.world.east,&estimate.world.north,&estimate.world.up};
                // Smooth both observed and predicted coordinates. A sender
                // prediction is correlated with earlier observations: retain
                // the prior uncertainty rather than claiming new information.
                for(size_t axis=0;axis<3;++axis) {
                    const double gain=1./(1.+measurement_variance_[axis]/prior[axis]);
                    *filtered[axis]+=gain*(measured[axis]-*filtered[axis]);
                    estimate.variance[axis]=observed?measurement_variance_[axis]*gain:prior[axis];
                }
                estimate.seen=raw.seen;
            }
            // Re-reading one snapshot (GUI frames or console queries) must not
            // count the same received observation multiple times.
            const BasePoint relative{estimate.world.east-origin.east,
                estimate.world.north-origin.north,estimate.world.up-origin.up};
            if (!gui_point(estimate.world) || !gui_point(relative)) {
                estimates_.erase(it);continue;
            }
            point.world=estimate.world;point.relative=relative;
        }
        return result;
    }
};
} // namespace c2::rdr
