#pragma once

#include "c2/protocol.hpp"
#include "c2/command_identity.hpp"
#include "c2/protocol_validation.hpp"

#include <cstdint>
#include <mutex>

namespace c2 {
struct ObservationCommandConfig {
    ObservationTurretLimits limits;
    std::uint64_t command_validity_us{};
    std::uint32_t first_command_id{1};
    std::uint32_t first_sequence{1};
};

class ObservationCommandService final {
public:
    explicit ObservationCommandService(ObservationCommandConfig config);

    [[nodiscard]] ObservationTurretCommand create(
        ObservationTurretCommandType type,
        float target_pan_deg,
        float target_tilt_deg,
        std::uint64_t now_us);
    void assign_identity(DevelopmentPoseCommand& command) { identity_.assign(command); }

private:
    ObservationCommandConfig config_;
    std::mutex mutex_;
    CommandIdentity identity_;
};
}  // namespace c2
