#ifndef A_STAR_MAP_GEN_H_
#define A_STAR_MAP_GEN_H_

#include <cstdint>
#include <vector>
#include "rviz.hpp"

struct DenseMap
{
    float x_range[2];
    float y_range[2];
    int row;
    int col;
    std::vector<int8_t> grid_status;
}; // struct DenseMap

struct MapGen
{
    MapGen(float x_range_from, float x_range_to, float y_range_from, float y_range_to, int row, int col);
    void add_obstacle(float center_x, float center_y, float width, float length);
    DenseMap map;
}; // class MapGen


void to_rviz_map(const DenseMap& map, rviz::GridMap2d& rviz_map);

#endif // A_STAR_MAP_GEN_H_
