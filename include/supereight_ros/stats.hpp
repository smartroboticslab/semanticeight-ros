// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_STATS_HPP
#define SE_STATS_HPP

#include <map>
#include <string>
#include <vector>

namespace se {

/** Store per-frame statistics for several program sections, e.g. fusion or planning and write them
 * to TSV files.
 *
 * \warning Valid section and statistic names must be defined in se::section_stat_names in
 * stats.cpp.
 */
class Stats {
    public:
    Stats();

    Stats(const std::string& stat_dir);

    void newFrame(const std::string& section);

    void sample(const std::string& section, const std::string& stat_name, double stat_value);

    void sampleTimeNow(const std::string& section, const std::string& stat_name);

    double get(const std::string& section, const std::string& stat_name) const;

    void writeFrame(const std::string& section) const;

    void print(const std::string& section, const std::vector<std::string>& stat_names) const;



    private:
    /** Statistics for a single frame. */
    typedef std::map<std::string, double> FrameStats;
    /** Statistics for all frames of a single section, e.g. Planning. */
    typedef std::vector<FrameStats> SectionStats;

    std::map<std::string, SectionStats> stats_;
    std::map<std::string, std::string> filenames_;
    double start_time_;
};

} // namespace se

#endif // SE_STATS_HPP
