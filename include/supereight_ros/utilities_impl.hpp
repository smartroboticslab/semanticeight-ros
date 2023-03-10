// SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

namespace se {
template<typename T, typename GetTimestampF>
bool get_closest_element(const boost::circular_buffer<T>& buffer,
                         const ros::Time& query_timestamp,
                         const double threshold,
                         GetTimestampF get_timestamp,
                         T& closest_element)
{
    // Find the minimum time difference.
    double min_time_diff = std::numeric_limits<double>::infinity();
    for (const auto e : buffer) {
        const double time_diff = std::fabs((get_timestamp(e) - query_timestamp).toSec());

        if (time_diff <= min_time_diff) {
            min_time_diff = time_diff;
            closest_element = e;
        }
        else {
            // The time difference started increasing which means the minimum has
            // been found, finish early.
            break;
        }
    }

    if (min_time_diff <= threshold) {
        return true;
    }
    else {
        return false;
    }
}
} // namespace se
