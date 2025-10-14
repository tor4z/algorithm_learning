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

    rviz::VehState2d state;
    state.x = 10;
    state.y = 2;
    state.heading = 0.5f;
    state.steer_angle = 0.2f;
    state.wheel_radius = 0.4f;
    state.wheel_base = 2.8f;
    state.wheel_track = 1.9f;
    state.wheel_width = 0.3f;

    viz->draw_gridmap2d("test/map", rviz_map);
    viz->draw_vehicle2d("test/veh", state);
    while (!viz->closed()) {
        viz->render();
    }

    return 0;
}
