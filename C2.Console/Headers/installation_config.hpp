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

// Common installation positions (forward,left,up) used by the map and FCS
// assignment calculation. Raw RDR targets are first rotated with their observation
// PAN/TILT by sensor_to_base; translate that result, then subtract FCS position.
struct InstallationConfig {
    std::array<double, 3> rdr_position{}, eff_position{};
    // Camera-only metadata, in GUI XYZ metres; never used by device control.
    std::array<double, 3> rdr_zero{}, fcs_zero{};
    std::array<double, 3> ranges{1.5, 10.0, 3.0};
    std::array<double, 3> rdr_fov{120.0, 45.0, 0.0};
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

inline std::array<double, 3> camera_triple(const nlohmann::json& value) {
    if (!value.is_array() || value.size()!=3)
        throw std::invalid_argument("Camera setting requires three numbers");
    std::array<double,3> result{};
    for (size_t i=0;i<3;++i) {
        if (!value[i].is_number()) throw std::invalid_argument("Camera setting must be numeric");
        result[i]=value[i].get<double>();
        if (!std::isfinite(result[i]) || std::abs(result[i])>std::numeric_limits<float>::max())
            throw std::invalid_argument("Camera setting must be finite and fit float");
    }
    return result;
}
inline void validate_camera_settings(const InstallationConfig& value) {
    (void)camera_triple(value.rdr_zero); (void)camera_triple(value.fcs_zero);
    (void)camera_triple(value.ranges);
    (void)camera_triple(value.rdr_fov);
    if (value.rdr_fov[0]<=0 || value.rdr_fov[0]>360 || value.rdr_fov[1]<=0 || value.rdr_fov[1]>180 || value.rdr_fov[2]!=0)
        throw std::invalid_argument("RDR horizontal FOV must be (0,360], vertical (0,180] degrees");
    if (value.ranges[0]<0 || value.ranges[1]<value.ranges[0] || value.ranges[2]<0)
        throw std::invalid_argument("Distances must be nonnegative and RDR maximum >= minimum");
}

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
    if (config.contains("camera")) {
        const auto& camera=config.at("camera");
        if (!camera.is_object()) throw std::invalid_argument("camera must be an object");
        if (camera.contains("rdr_zero")) result.rdr_zero=camera_triple(camera.at("rdr_zero"));
        if (camera.contains("fcs_zero")) result.fcs_zero=camera_triple(camera.at("fcs_zero"));
        if (camera.contains("ranges")) result.ranges=camera_triple(camera.at("ranges"));
        result.rdr_fov=camera_triple(nlohmann::json::array({
            camera.value("rdr_horizontal_fov_deg", nlohmann::json(120.0)),
            camera.value("rdr_vertical_fov_deg", nlohmann::json(45.0)), 0}));
    }
    validate_camera_settings(result);
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
    validate_camera_settings(value);
    settings["camera"]["rdr_zero"]=value.rdr_zero;
    settings["camera"]["fcs_zero"]=value.fcs_zero;
    settings["camera"]["ranges"]=value.ranges;
    settings["camera"]["rdr_horizontal_fov_deg"]=value.rdr_fov[0];
    settings["camera"]["rdr_vertical_fov_deg"]=value.rdr_fov[1];
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
