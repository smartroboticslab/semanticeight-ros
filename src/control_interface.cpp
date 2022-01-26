// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/control_interface.hpp"

#include <algorithm>
#include <cctype>
#include <map>

namespace se {

std::string control_interface_to_str(ControlInterface c)
{
    switch (c) {
    case ControlInterface::Habitat:
        return "Habitat";
    case ControlInterface::RotorS:
        return "RotorS";
    case ControlInterface::SRL:
        return "SRL";
    default:
        throw std::invalid_argument("invalid ControlInterface value: "
                                    + std::to_string(static_cast<int>(c)));
    }
}



ControlInterface str_to_control_interface(const std::string& s)
{
    std::string s_lower = s;
    std::transform(s.begin(), s.end(), s_lower.begin(), [](unsigned char c) -> unsigned char {
        return std::tolower(c);
    });

    if (s_lower == "habitat") {
        return ControlInterface::Habitat;
    }
    else if (s_lower == "rotors") {
        return ControlInterface::RotorS;
    }
    else if (s_lower == "srl") {
        return ControlInterface::SRL;
    }
    else {
        throw std::invalid_argument("invalid ControlInterface name: " + s);
    }
}

} // namespace se
