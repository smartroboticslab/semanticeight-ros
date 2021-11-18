# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
# # SPDX-License-Identifier: CC0-1.0

.PHONY: format
format:
	find include src test -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format-10 -style=file -i {} \;

