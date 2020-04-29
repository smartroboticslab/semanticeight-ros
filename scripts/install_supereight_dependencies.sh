#!/bin/sh
# install_supereight_dependencies.sh
#
# SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

set -eu
IFS="$(printf '%b_' '\t\n')" ; IFS="${IFS%_}"

# Install binary dependencies
DEBIAN_FRONTEND=noninteractive apt-get -y install \
	build-essential git cmake libeigen3-dev libopencv-dev

# Find the temporary file directory
tmpdir="${TMPDIR:-/tmp}"

# Install Sophus from source
git clone https://github.com/strasdat/Sophus.git "${tmpdir}/Sophus";
mkdir -p "${tmpdir}/Sophus/build";
cd "${tmpdir}/Sophus/build";
git checkout v1.0.0;
sed -i \
	's/option(BUILD_TESTS "Build tests." ON)/option(BUILD_TESTS "Build tests." OFF)/' \
	"${tmpdir}/Sophus/build/CMakeLists.txt";
cmake -DCMAKE_BUILD_TYPE=Release ..;
make install;

