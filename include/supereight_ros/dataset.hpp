// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef DATASET_HPP
#define DATASET_HPP

#include <string>

namespace se {

enum class Dataset {
    Real = 0x10,
    Gazebo = 0x20,
    Habitat = 0x40,
    Matterport3D = 0x41,
    Replica = 0x42,
};

Dataset base_dataset(Dataset d);

std::string dataset_to_str(Dataset d);

Dataset str_to_dataset(const std::string& s);

} // namespace se

#endif
