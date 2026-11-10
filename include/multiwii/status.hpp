#pragma once

#include <algorithm>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace multiwii {

// MSP_STATUS bits index the advertised box list on Cleanflight/Betaflight.
// They are not permanent MSP_BOXIDS (e.g. FAILSAFE's permanent ID is 27).
inline std::size_t arm_index(const std::vector<std::string>& names) {
    const auto arm = std::find(names.begin(), names.end(), "ARM");
    if(arm == names.end() || std::count(names.begin(), names.end(), "ARM") != 1 ||
       std::distance(names.begin(), arm) >= 32) {
        throw std::runtime_error("MSP box table must contain one ARM entry within the first 32 boxes");
    }
    return static_cast<std::size_t>(std::distance(names.begin(), arm));
}

inline bool active(const std::vector<std::string>& names,
                   const std::set<std::size_t>& flags, const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    return it != names.end() && flags.count(static_cast<std::size_t>(it - names.begin()));
}

inline std::string flight_mode(const std::vector<std::string>& names,
                               const std::set<std::size_t>& flags) {
    // Select the highest-priority active mode.
    for(const auto* name : {"FAILSAFE", "GPS RESCUE", "GPS HOME", "GPS HOLD",
                           "PASSTHRU", "HORIZON", "ANGLE"}) {
        if(active(names, flags, name)) return name;
    }
    return "ACRO";  // No self-levelling/navigation mode is active, even if disarmed.
}

}  // namespace multiwii
