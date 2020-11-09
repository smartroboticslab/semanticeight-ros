#!/bin/sh
# install_supereight_dependencies.sh
#
# SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

set -eu
IFS="$(printf '%b_' '\t\n')" ; IFS="${IFS%_}"

echo 'This script requires root privileges to install packages on your system'

# Install binary dependencies
DEBIAN_FRONTEND=noninteractive sudo apt-get -y install \
	build-essential git cmake libeigen3-dev libopencv-dev

