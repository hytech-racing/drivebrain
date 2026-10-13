#pragma once

#include <iostream>
#include <vector> 
#include <cmath>
#include <algorithm>

typedef std::pair<double, double> table_entry;
typedef std::vector<table_entry> table_type;

class InterpolatingTable {

    public: 

        InterpolatingTable(const table_type& table) : _table(table) {
            std::sort(_table.begin(), _table.end(), [](const table_entry& a, const table_entry& b) {
                return a.first < b.first;
            });
        }

        // Returns a pair of [interpolatedValue, distanceToNearestPoint].
        // We use the distanceToNearestPoint for scaling standard deviations for load cell measurements
        std::pair<double, double> interpolate(double x) const {
            if (_table.empty()) {
                throw std::runtime_error("Interpolation table is empty.");
            }

            if (std::isnan(x)) {
                throw std::runtime_error("Input value to interpolating table is NaN.");
            }

            if (x <= _table.front().first) {
                return {_table.front().second, _table.front().first - x};
            }

            if (x >= _table.back().first) {
                return {_table.back().second, x - _table.back().first};
            }

            auto it = std::lower_bound(_table.begin(), _table.end(), x, [](const table_entry& entry, double value) {
                return entry.first < value;
            });

            auto& [x0, y0] = *(it - 1);
            auto& [x1, y1] = *it;

            double slope = (y1 - y0) / (x1 - x0);
            double interpolation = y0 + slope * (x - x0);
            double nearestDist = std::min(std::abs(x - x0), std::abs(x1 - x));
            return {interpolation, nearestDist};
        }

    private: 
        
        table_type _table;


};