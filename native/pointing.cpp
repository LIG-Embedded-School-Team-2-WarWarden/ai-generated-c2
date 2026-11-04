#include "c2mvp/domain.hpp"
#include <cmath>
#include <numbers>
namespace c2mvp {
std::pair<double, double> RelativePointingSolver::solve(const Object &o, const Installation &obs,
                                                        const Installation &eff) const {
    std::array<double, 3> p{o.x + obs.offset[0], o.y + obs.offset[1], o.z + obs.offset[2]}, delta{},
        local{};
    for (int i = 0; i < 3; ++i) {
        delta[i] = obs.position[i] - eff.position[i];
        for (int k = 0; k < 3; ++k)
            delta[i] += obs.rotation[3 * i + k] * p[k];
    }
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k)
            local[i] += eff.rotation[3 * k + i] * delta[k];
        // The mounting offset is expressed in effector-local axes.
        local[i] -= eff.offset[i];
    }
    double h = std::hypot(local[0], local[1]);
    require(h >= 1e-8 && finite(h) && finite(local[2]), "undefined pointing angle");
    return {std::atan2(local[1], local[0]) * 180 / std::numbers::pi,
            std::atan2(local[2], h) * 180 / std::numbers::pi};
}
} // namespace c2mvp
