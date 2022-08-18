#!/usr/bin/env python3
import argparse
import os
import os.path
import sys

from typing import Dict, List

class SequenceStats:
    # Desired map resolution is 0.04 meters.
    _map = {size * 0.04: size for size in [128, 256, 512, 1024, 2048, 4096, 8192]}

    def __init__(self, line: str, header: Dict[str, int]) -> None:
        self._header = header
        self._fields = line.split("\t")
        self.dirname = self._unexpand_tilde(self._fields[header["Dirname"]])
        self.name = os.path.basename(self.dirname)
        self.filename = self.dirname + "/" + self.name + ".glb"
        max_dim = max(self._get_floats("Range"))
        self.map_dim = self._dim(max_dim)
        self.map_res = self._res(self.map_dim)
        self.center_W = self._get_floats("Center")
        self.center_M = 3 * [0.5 * self.map_dim]
        self.t_MW = [m - w for m, w in zip(self.center_M, self.center_W)]
        self.t_MW_factor = [x / self.map_dim for x in self.t_MW]

    def _get_floats(self, name: str) -> List[str]:
        return [float(self._fields[self._header[x]]) for x in sorted(self._header) if name in x]

    @staticmethod
    def _unexpand_tilde(path: str) -> str:
        home = os.environ["HOME"]
        if path.startswith(home):
            return path.replace(home, "~", 1)
        return path

    @staticmethod
    def _dim(dimension: float) -> float:
        for map_dim in SequenceStats._map:
            if dimension <= map_dim:
                return map_dim
        raise RuntimeError(f"Map dimensions too large: {dimension}")

    @staticmethod
    def _res(dimension: float) -> int:
        return SequenceStats._map[dimension]

    def config(self) -> str:
        return f'''# SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
# SPDX-License-Identifier: CC0-1.0

supereight_ros:
  visualization_max_z: 3.5
  max_exploration_time: 2700 # 45 minutes
  dataset: "Matterport3D"

habitat:
  scene_file: "{self.filename}"
  initial_T_HB: [0.0, 0.0, 1.4]

supereight:
  general:
    sequence_name:              "matterport3d_{self.name}"
  map:
    size:                       {self.map_res}
    dim:                        {self.map_dim}
    t_MW_factor:                {self.t_MW_factor}
'''

    def write_config(self, directory: str) -> None:
        os.makedirs(directory, exist_ok=True)
        filename = directory + "/" + self.name + ".yaml"
        with open(filename, "w") as f:
            f.write(self.config())


def parse_tsv_header(line: str) -> Dict[str, int]:
    fields = line.split("\t")
    return {x:i for i, x in enumerate(fields)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="""Read statistics for
            Matterport3D sequences in TSV format from standard input and
            write a semanticeight config for each one inside DIRECTORY. Pipe
            the output of mp3d-sequence-stats.py from dataset-tools to this
            script.""")
    parser.add_argument("dir", metavar="DIRECTORY", nargs="?", default=".",
            help="The DIRECTORY where config files will be written.")
    return parser.parse_args()


if __name__ == "__main__":
    try:
        args = parse_args()
        header = parse_tsv_header(sys.stdin.readline())
        for line in sys.stdin:
            SequenceStats(line, header).write_config(args.dir)
    except KeyboardInterrupt:
        pass
