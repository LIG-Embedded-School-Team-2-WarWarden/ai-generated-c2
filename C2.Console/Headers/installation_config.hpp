#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

// Common installation positions (forward,left,up) used by the map and FCS
// assignment calculation. Incoming RDR targets are PAN-fixed but RDR-relative;
// translate by RDR position, then subtract FCS position. Do not rotate again.
struct InstallationConfig {
    std::array<double, 3> rdr_position{}, eff_position{};
    std::string observer_id, effector_id;
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
