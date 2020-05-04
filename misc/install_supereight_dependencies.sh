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

# Find the temporary file directory
tmpdir="${TMPDIR:-/tmp}"

# Install Sophus from source
rm -rf "${tmpdir}/Sophus";
git clone https://github.com/strasdat/Sophus.git "${tmpdir}/Sophus";
mkdir -p "${tmpdir}/Sophus/build";
cd "${tmpdir}/Sophus/build";
git checkout v1.0.0;
sed -i \
	's/option(BUILD_TESTS "Build tests." ON)/option(BUILD_TESTS "Build tests." OFF)/' \
	"${tmpdir}/Sophus/CMakeLists.txt";
cmake -DCMAKE_BUILD_TYPE=Release ..;
sudo make install;

