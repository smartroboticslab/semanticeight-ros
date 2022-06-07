// SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/stats.hpp"

#include <fstream>
#include <ros/ros.h>

#include "supereight_ros/filesystem.hpp"

/** A map from section names to valid statistic names in printing order. Couldn't make it a static
 * const member of se::Stats because the complier complained. */
static const std::map<std::string, std::vector<std::string>> section_stat_names = {
    {"fusion", {"Frame",       "Timestamp",       "Preprocessing",
                "Tracking",    "Integration",     "Object integration",
                "Rendering",   "Visualization",   "Total",
                "Free volume", "Occupied volume", "Explored volume",
                "RAM usage",   "t_WB x",          "t_WB y",
                "t_WB z",      "q_WB x",          "q_WB y",
                "q_WB z",      "q_WB w",          "Number of objects",
                "Scale 0",     "Scale 1",         "Scale 2",
                "Scale 3"}},
    {"planning",
     {"Planning iteration",
      "Timestamp",
      "Planning time",
      "Goal utility",
      "Goal entropy utility",
      "Goal object LoD utility",
      "Goal object dist utility",
      "Goal object compl utility",
      "Goal entropy gain",
      "Goal object LoD gain",
      "Goal object dist gain",
      "Goal object compl gain",
      "Goal path time",
      "Exploration dominant",
      "Goal t_WB x",
      "Goal t_WB y",
      "Goal t_WB z",
      "Goal q_WB x",
      "Goal q_WB y",
      "Goal q_WB z",
      "Goal q_WB w"}},
    {"network", {"Timestamp", "Network time"}}};



template<typename T>
int append_tsv_line(const std::string& filename, const std::vector<T>& line_data)
{
    std::ofstream f(filename, std::ios::app);
    if (f.good()) {
        f << std::setprecision(6) << std::fixed;
        for (size_t i = 0; i < line_data.size(); i++) {
            f << line_data[i] << (i == line_data.size() - 1 ? "\n" : "\t");
        }
        f.close();
    }
    return !f.good();
}



namespace se {

Stats::Stats() : start_time_(-1.0)
{
    for (const auto& e : section_stat_names) {
        const std::string& section = e.first;
        // Initialize the section data to an empty vector.
        stats_.emplace(section, 0);
        filenames_[section] = "";
    }
}

Stats::Stats(const std::string& stat_dir) : Stats()
{
    if (!stat_dir.empty()) {
        stdfs::create_directories(stat_dir);
        for (const auto& e : section_stat_names) {
            const std::string& section = e.first;
            const std::vector<std::string>& stat_names = e.second;
            filenames_[section] = stat_dir + "/stats_" + section + ".tsv";
            if (append_tsv_line(filenames_.at(section), stat_names)) {
                ROS_WARN_ONCE("Can't write %s statistics to \"%s\"",
                              section.c_str(),
                              filenames_.at(section).c_str());
            }
            else {
                ROS_INFO("Writing %s statistics to \"%s\"",
                         section.c_str(),
                         filenames_.at(section).c_str());
            }
        }
    }
}

void Stats::newFrame(const std::string& section)
{
    if (start_time_ < 0.0) {
        start_time_ = ros::WallTime::now().toSec();
    }
    try {
        // Add default-initialized FrameStats to the back of the vector.
        stats_.at(section).emplace_back();
        // Initialize each frame stat to zero.
        for (const std::string& name : section_stat_names.at(section)) {
            stats_.at(section).back()[name] = 0.0;
        }
    }
    catch (const std::out_of_range& e) {
        throw std::out_of_range("invalid stat section \"" + section + "\"");
    }
}

void Stats::sample(const std::string& section, const std::string& stat_name, double stat_value)
{
    try {
        FrameStats& frame_stats = stats_.at(section).back();
        try {
            frame_stats.at(stat_name) = stat_value;
        }
        catch (const std::out_of_range& e) {
            throw std::out_of_range("invalid stat name \"" + stat_name + "\" for section \""
                                    + section + "\"");
        }
    }
    catch (const std::out_of_range& e) {
        throw std::out_of_range("invalid stat section \"" + section + "\"");
    }
}

void Stats::sampleTimeNow(const std::string& section, const std::string& stat_name)
{
    sample(section, stat_name, ros::WallTime::now().toSec() - start_time_);
}

double Stats::get(const std::string& section, const std::string& stat_name) const
{
    try {
        const FrameStats& frame_stats = stats_.at(section).back();
        try {
            return frame_stats.at(stat_name);
        }
        catch (const std::out_of_range& e) {
            throw std::out_of_range("invalid stat name \"" + stat_name + "\" for section \""
                                    + section + "\"");
        }
    }
    catch (const std::out_of_range& e) {
        throw std::out_of_range("invalid stat section \"" + section + "\"");
    }
}

void Stats::writeFrame(const std::string& section) const
{
    try {
        if (filenames_.at(section).empty()) {
            return;
        }
        const FrameStats& frame_stats = stats_.at(section).back();
        const std::vector<std::string> stat_names = section_stat_names.at(section);
        std::vector<double> stat_values(stat_names.size());
        std::transform(stat_names.begin(),
                       stat_names.end(),
                       stat_values.begin(),
                       [&](const std::string& s) -> double { return frame_stats.at(s); });
        append_tsv_line(filenames_.at(section), stat_values);
    }
    catch (const std::out_of_range& e) {
        throw std::out_of_range("invalid stat section \"" + section + "\"");
    }
}

void Stats::print(const std::string& section, const std::vector<std::string>& stat_names) const
{
    for (const auto& stat_name : stat_names) {
        ROS_INFO("%-25s %.5f s", stat_name.c_str(), get(section, stat_name));
    }
}

} // namespace se
