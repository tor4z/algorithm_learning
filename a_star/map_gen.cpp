#include "map_gen.hpp"
#include <algorithm>
#include <cassert>

MapGen::MapGen(float x_range_from, float x_range_to, float y_range_from, float y_range_to, int row, int col)
{
    assert(x_range_to > x_range_from);
    assert(y_range_to > y_range_from);
    assert(row > 0);
    assert(col > 0);

    map.x_range[0] = x_range_from;
    map.x_range[1] = x_range_to;
    map.y_range[0] = y_range_from;
    map.y_range[1] = y_range_to;
    map.row = row;
    map.col = col;
    map.grid_status.resize(row * col, 0);
}

void MapGen::add_obstacle(float center_x, float center_y, float width, float length)
{
    const auto half_width{width / 2.0f};
    const auto half_length{length / 2.0f};

    const auto x_res{(map.x_range[1] - map.x_range[0]) / map.row};
    const auto y_res{(map.y_range[1] - map.y_range[0]) / map.col};
    const auto x_inc{std::min(x_res, half_length)};
    const auto y_inc{std::min(y_res, half_width)};
    for (float x = center_x - half_length; x < center_x + half_length; x += x_inc) {
        if (x >= map.x_range[1] && x <= map.x_range[0]) {
            continue;
        }
        const auto r{map.row - 1 - static_cast<int>((x - map.x_range[0]) / x_res)};
        for (float y = center_y - half_width; y < center_y + half_width; y += y_inc) {
            if (y >= map.y_range[1] && y <= map.y_range[0]) {
                continue;
            }
            const auto c{map.col - 1 - static_cast<int>((y - map.y_range[0]) / y_res)};
            map.grid_status[r * map.col + c] = 1;
        }
    }
}

void to_rviz_map(const DenseMap& map, rviz::GridMap2d& rviz_map)
{
    rviz_map.range_x[0] = map.x_range[0];
    rviz_map.range_x[1] = map.x_range[1];
    rviz_map.range_y[0] = map.y_range[0];
    rviz_map.range_y[1] = map.y_range[1];
    rviz_map.row = map.row;
    rviz_map.col = map.col;

    rviz_map.occupied_grid.reserve(map.grid_status.size());
    for (size_t i = 0; i < map.grid_status.size(); ++i) {
        if (map.grid_status.at(i) > 0) rviz_map.occupied_grid.push_back(i);
    }
}
