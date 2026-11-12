#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include <windows.h>

#include <nlohmann/json.hpp>
#include "fcs_angles.hpp"

inline nlohmann::json fcs_calibration_json(const FcsCalibration& c) {
    return {{"pan_offset_deg",c.pan_offset_deg},{"tilt_offset_deg",c.tilt_offset_deg},
        {"yaw_deg",c.yaw_deg},{"pitch_deg",c.pitch_deg},{"roll_deg",c.roll_deg}};
}
inline FcsCalibration parse_fcs_calibration(const nlohmann::json& json) {
    if(!json.is_object())throw std::invalid_argument("FCS calibration must be an object");
    FcsCalibration result;
    const auto read=[&](const char* key,double& value) {
        if(!json.contains(key))return;
        if(!json.at(key).is_number())throw std::invalid_argument("FCS calibration must contain numeric degrees");
        value=json.at(key).get<double>();
    };
    read("pan_offset_deg",result.pan_offset_deg);read("tilt_offset_deg",result.tilt_offset_deg);
    read("yaw_deg",result.yaw_deg);read("pitch_deg",result.pitch_deg);read("roll_deg",result.roll_deg);
    validate_fcs_calibration(result);
    return result;
}

// Common installation positions (forward,left,up) used by the map and FCS
// assignment calculation. Raw RDR targets are first rotated with their observation
// PAN/TILT by sensor_to_base; translate that result, then subtract FCS position.
struct InstallationConfig {
    std::array<double, 3> rdr_position{}, eff_position{};
    FcsCalibration fcs_calibration{};
    std::string observer_id{"RDR-1"}, effector_id{"FCS-1"};
};

namespace installation_config_detail {
inline std::string selected_id(const nlohmann::json& config, const char* key) {
    if (!config.contains(key) || !config.at(key).is_string() ||
        config.at(key).get_ref<const std::string&>().empty())
        throw std::invalid_argument(std::string("Installation config: '") + key +
                                    "' must be a nonempty device ID string");
    return config.at(key).get<std::string>();
}

inline std::array<double, 3> position(const nlohmann::json& devices,
                                     const std::string& id, const char* role) {
    if (!devices.contains(id) || !devices.at(id).is_object())
        throw std::invalid_argument("Installation config: missing device '" + id + "'");
    const auto& device = devices.at(id);
    if (!device.contains("role") || !device.at("role").is_string() ||
        device.at("role").get_ref<const std::string&>() != role)
        throw std::invalid_argument("Installation config: device '" + id +
                                    "' must have role " + role);
    if (!device.contains("position") || !device.at("position").is_array() ||
        device.at("position").size() != 3)
        throw std::invalid_argument("Installation config: device '" + id +
                                    "' position must contain exactly three numbers");
    std::array<double, 3> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        const auto& value = device.at("position").at(i);
        if (!value.is_number())
            throw std::invalid_argument("Installation config: device '" + id +
                                        "' position must contain numbers");
        result[i] = value.get<double>();
        if (!std::isfinite(result[i]) ||
            std::abs(result[i]) > std::numeric_limits<float>::max())
            throw std::invalid_argument("Installation config: device '" + id +
                                        "' position must be finite and fit the map coordinates");
    }
    return result;
}
} // namespace installation_config_detail

inline InstallationConfig parse_installation_config(const nlohmann::json& config) {
    if (!config.is_object() || !config.contains("devices") ||
        !config.at("devices").is_object())
        throw std::invalid_argument("Installation config: 'devices' must be an object");
    const auto& devices = config.at("devices");
    InstallationConfig result;
    result.observer_id = installation_config_detail::selected_id(config, "observer");
    result.rdr_position = installation_config_detail::position(devices, result.observer_id, "OBS");
    if (config.contains("effector")) {
        result.effector_id = installation_config_detail::selected_id(config, "effector");
    } else {
        for (const auto& [id, device] : devices.items()) {
            if (device.is_object() && device.contains("role") && device.at("role").is_string() &&
                device.at("role").get_ref<const std::string&>() == "EFF") {
                if (!result.effector_id.empty())
                    throw std::invalid_argument(
                        "Installation config: multiple EFF devices; specify 'effector'");
                result.effector_id = id;
            }
        }
        if (result.effector_id.empty())
            throw std::invalid_argument("Installation config: no EFF device was configured");
    }
    result.eff_position = installation_config_detail::position(devices, result.effector_id, "EFF");
    const auto& effector=devices.at(result.effector_id);
    if(effector.contains("calibration"))result.fcs_calibration=parse_fcs_calibration(effector.at("calibration"));
    return result;
}

inline InstallationConfig load_installation_config(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input)
        throw std::invalid_argument("Cannot open installation config: " + path.string());
    try {
        return parse_installation_config(nlohmann::json::parse(input));
    } catch (const nlohmann::json::exception& error) {
        throw std::invalid_argument("Invalid installation config '" + path.string() +
                                    "': " + error.what());
    }
}

// Runtime settings share the --config document without changing its network keys.
inline nlohmann::json read_installation_settings_document(
    const std::filesystem::path& path, bool allow_missing = false) {
    if (allow_missing && !std::filesystem::exists(path)) return nlohmann::json::object();
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open settings: " + path.string());
    auto document = nlohmann::json::parse(input);
    if (!document.is_object()) throw std::invalid_argument("Settings must be a JSON object");
    return document;
}

inline InstallationConfig load_installation_settings(
    const std::filesystem::path& path, bool allow_missing = false) {
    const auto document = read_installation_settings_document(path, allow_missing);
    return document.contains("installation") ?
        parse_installation_config(document.at("installation")) : InstallationConfig{};
}

inline void save_installation_settings(const std::filesystem::path& path,
                                      const InstallationConfig& value,
                                      bool allow_missing = false) {
    auto document = read_installation_settings_document(path, allow_missing);
    auto settings = document.value("installation", nlohmann::json::object());
    if (document.contains("installation")) (void)parse_installation_config(settings);
    settings["observer"] = value.observer_id;
    settings["effector"] = value.effector_id;
    auto& devices = settings["devices"];
    devices[value.observer_id]["role"] = "OBS";
    devices[value.observer_id]["position"] = value.rdr_position;
    devices[value.effector_id]["role"] = "EFF";
    devices[value.effector_id]["position"] = value.eff_position;
    auto& calibration=devices[value.effector_id]["calibration"];
    if(calibration.is_null())calibration=nlohmann::json::object();
    calibration.update(fcs_calibration_json(value.fcs_calibration));
    (void)parse_installation_config(settings);
    document["installation"] = std::move(settings);

    // Replace only after a complete write; a failed save keeps the old file.
    auto temporary = path;
    temporary += L".tmp." + std::to_wstring(GetCurrentProcessId());
    try {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) throw std::runtime_error("Cannot create settings temporary file: " + temporary.string());
        output << document.dump(2) << '\n';
        output.flush();
        if (!output) throw std::runtime_error("Cannot write settings: " + temporary.string());
        output.close();
        if (!output) throw std::runtime_error("Cannot close settings: " + temporary.string());
        if (!MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            throw std::system_error(GetLastError(), std::system_category(), "Cannot replace settings");
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw;
    }
}
