//
// Created by anna on 16/05/19.
//

#ifndef EXPLORATION_WS_SUPEREIGHT_UTILS_HPP
#define EXPLORATION_WS_SUPEREIGHT_UTILS_HPP

#include <GL/glut.h>
struct float3 {
  float x, y, z;
};
inline float3 make_float3(float x, float y, float z) {

  float3 val;
  val.x = x;
  val.y = y;
  val.z = z;
  return val;
}
struct uint2 {
  unsigned int x;
  unsigned int y;
};

struct uchar4 {
  unsigned char x, y, z, w;
};
inline uint2 make_uint2(unsigned int x, unsigned int y) {
  uint2 val;
  val.x = x;
  val.y = y;
  return val;
}

void storeStats(int frame,
                std::chrono::time_point<std::chrono::steady_clock> *timings,
                float3 pos, bool tracked,
                bool integrated) {
  Stats.sample("frame", frame, PerfStats::FRAME);
  Stats.sample("image conversion ",
               std::chrono::duration<double>(timings[1] - timings[0]).count(),
               PerfStats::TIME);
  Stats.sample("pose interpolation",
               std::chrono::duration<double>(timings[2] - timings[1]).count(),
               PerfStats::TIME);
  Stats.sample("preprocessing",
               std::chrono::duration<double>(timings[3] - timings[2]).count(),
               PerfStats::TIME);
  Stats.sample("tracking",
               std::chrono::duration<double>(timings[4] - timings[3]).count(),
               PerfStats::TIME);
  Stats.sample("integration",
               std::chrono::duration<double>(timings[5] - timings[4]).count(),
               PerfStats::TIME);
  Stats.sample("raycasting",
               std::chrono::duration<double>(timings[6] - timings[5]).count(),
               PerfStats::TIME);
  Stats.sample("computation",
               std::chrono::duration<double>(timings[8] - timings[1]).count(),
               PerfStats::TIME);
  Stats.sample("total",
               std::chrono::duration<double>(timings[8] - timings[0]).count(),
               PerfStats::TIME);
  Stats.sample("X", pos.x, PerfStats::DISTANCE);
  Stats.sample("Y", pos.y, PerfStats::DISTANCE);
  Stats.sample("Z", pos.z, PerfStats::DISTANCE);
  Stats.sample("tracked", tracked, PerfStats::INT);
  Stats.sample("integrated", integrated, PerfStats::INT);
  Stats.sample("rendering",
               std::chrono::duration<double>(timings[7] - timings[6]).count(),
               PerfStats::TIME);
  Stats.sample("map visualization",
               std::chrono::duration<double>(timings[8] - timings[7]).count(),
               PerfStats::TIME);
}
#endif //EXPLORATION_WS_SUPEREIGHT_UTILS_HPP
