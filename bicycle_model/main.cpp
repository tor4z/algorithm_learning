#define BICYCLE_IMPLEMENTATION
#include "bicycle.hpp"

#define HLOG_IMPLEMENTATION
#include "hlog.h"

#define RVIZ_TARGET_FPS 50
#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


#define WHEEL_TRACE     1.9f    // meter
#define WHEEL_WIDTH     0.3f    // meter
#define WHEEL_RADIUS    0.4f    // meter
#define WHEEL_BASE      2.8f    // meter
#define MIN_STEER       -0.5f   // rad
#define MAX_STEER       0.5f    // rad

int main()
{
    auto viz{rviz::Viz::instance()};
    bicycle::Bicycle::State state{0};
    bicycle::Bicycle::Config cfg{
        .wheel_base=WHEEL_BASE,
        .gc_to_back_axle=WHEEL_BASE/2,
        .max_steer=MAX_STEER,
        .min_steer=MIN_STEER
    };
    bicycle::Bicycle model(state, cfg);

    rviz::VehState2d rviz_state;
    rviz_state.heading = 0.0f;
    rviz_state.steer_angle = 0;
    rviz_state.x = 0;
    rviz_state.y = 0;
    rviz_state.wheel_base = WHEEL_BASE;
    rviz_state.wheel_track = WHEEL_TRACE;
    rviz_state.wheel_radius = WHEEL_RADIUS;
    rviz_state.wheel_width = WHEEL_WIDTH;

    int i{0};
    while (!viz->closed()) {
        const float accel{i < 10 ? 0.1f : 0.0f};
        const float steer_spd{i < 100 ? 0.1f : 
            model.state().steer_angle > 0.0f ? -0.01f : 0.01f
        };

        model.act(steer_spd, accel, 0.2f);
        rviz_state.x = model.state().x;
        rviz_state.y = model.state().y;
        rviz_state.steer_angle = model.state().steer_angle;
        rviz_state.heading = model.state().yaw;
        viz->draw_vehicle2d("test/veh", rviz_state);
        viz->draw_trj2d_point_("test/trj", rviz_state.x, rviz_state.y);

        viz->render();
        ++i;
    }
    return 0;
}
