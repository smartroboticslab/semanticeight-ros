// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef CONTROL_INTERFACE_HPP
#define CONTROL_INTERFACE_HPP

#include <string>

namespace se {

enum class ControlInterface { Habitat, RotorS, SRL };

std::string control_interface_to_str(ControlInterface c);

ControlInterface str_to_control_interface(const std::string& s);

} // namespace se

#endif
