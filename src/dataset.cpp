// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/dataset.hpp"

#include <algorithm>
#include <cctype>
#include <map>

namespace se {

Dataset base_dataset(Dataset d)
{
    return static_cast<Dataset>(static_cast<int>(d) & ~static_cast<int>(0xF));
}



std::string dataset_to_str(Dataset d)
{
    switch (d) {
    case Dataset::Real:
        return "Real";
    case Dataset::Gazebo:
        return "Gazebo";
    case Dataset::Habitat:
        return "Habitat";
    case Dataset::Matterport3D:
        return "Matterport3D";
    case Dataset::Replica:
        return "Replica";
    default:
        throw std::invalid_argument("invalid Dataset value: "
                                    + std::to_string(static_cast<int>(d)));
    }
}



Dataset str_to_dataset(const std::string& s)
{
    std::string s_lower = s;
    std::transform(s.begin(), s.end(), s_lower.begin(), [](unsigned char c) -> unsigned char {
        return std::tolower(c);
    });

    if (s_lower == "real") {
        return Dataset::Real;
    }
    else if (s_lower == "gazebo") {
        return Dataset::Gazebo;
    }
    else if (s_lower == "habitat") {
        return Dataset::Habitat;
    }
    else if (s_lower == "matterport3d") {
        return Dataset::Matterport3D;
    }
    else if (s_lower == "replica") {
        return Dataset::Replica;
    }
    else {
        throw std::invalid_argument("invalid Dataset name: " + s);
    }
}

} // namespace se
