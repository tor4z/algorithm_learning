#include <iostream>

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto viz{rviz::Viz::instance()};
    rviz::GridMap2d map;

    viz->draw_gridmap2d("test", map);
    while (!viz->closed()) {
        viz->render();
    }

    return 0;
}
