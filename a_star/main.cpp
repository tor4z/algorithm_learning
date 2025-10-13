#include "map_gen.hpp"

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto viz{rviz::Viz::instance()};
    rviz::GridMap2d rviz_map;

    MapGen mg(0, 10, 0, 10, 10, 10);
    mg.add_obstacle(2, 2, 1, 1);
    to_rviz_map(mg.map, rviz_map);

    viz->draw_gridmap2d("test", rviz_map);
    while (!viz->closed()) {
        viz->render();
    }

    return 0;
}
