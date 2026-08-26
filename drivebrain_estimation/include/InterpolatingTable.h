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

        double interpolate(double x) const {
            if (_table.empty()) {
                throw std::runtime_error("Interpolation table is empty.");
            }

            if (x <= _table.front().first) {
                return _table.front().second;
            }

            if (x >= _table.back().first) {
                return _table.back().second;
            }

            auto it = std::lower_bound(_table.begin(), _table.end(), x, [](const table_entry& entry, double value) {
                return entry.first < value;
            });

            auto& [x0, y0] = *(it - 1);
            auto& [x1, y1] = *it;

            double t = (x - x0) / (x1 - x0);
            return std::lerp(y0, y1, t);
        }

    private: 
        
        table_type _table;


};