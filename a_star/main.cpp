#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


struct State
{
    float x;
    float y;
    float heading;
}; // struct State

template<typename T>
T pow2(T v) { return v * v; }

template<typename T>
T pow3(T v) { return v * v * v; }

float euler_dist(const State& a, const State& b)
{
    return std::sqrt(pow2(a.x - b.x) + pow2(a.y - b.y));
}

class HybridAStar
{
public:
    HybridAStar(const State& init, const State& goal);
    ~HybridAStar() = default;
    bool search();
private:
    void find_neighbors(const State& current, std::vector<State>& neighbors);
    float neighbor_score(const State& neighbor);
    void kinematic_model(State& state, float steer_angle, float moves);

    State init_;
    State goal_;
}; // class HybridAStar

HybridAStar::HybridAStar(const State& init, const State& goal)
    : init_(init)
    , goal_(goal)
{}

bool HybridAStar::search()
{
    constexpr float short_distance{0.5f}; // meter

    std::vector<State> neighbors;
    State current{init_};
    int i = 0;
    auto viz{rviz::Viz::instance()};
    while (true) {
        if (i++ > 30) break;
        if (euler_dist(current, goal_) < short_distance) {
            break;
        }

        find_neighbors(current, neighbors);
        if (neighbors.empty()) {
            break;
        }

        State best_move{};
        float best_score{0};
        for (size_t i = 0; i < neighbors.size(); ++i) {
            const auto& neighbor{neighbors.at(i)};
            const auto this_score{neighbor_score(neighbor)};
            if (this_score > best_score) {
                best_score = this_score;
                best_move = neighbor;
            }
        }
        viz->draw_trj2d_point("test/trj", best_move.x, best_move.y);
        current = best_move;
    }

    return true;
}

float HybridAStar::neighbor_score(const State& neighbor)
{
    return 100 - (std::abs(neighbor.x - goal_.x) + std::abs(neighbor.y - goal_.y));
}

void HybridAStar::find_neighbors(const State& current, std::vector<State>& neighbors)
{
    constexpr float steer_start{-0.5f};
    constexpr float steer_end{0.5f};
    constexpr float steer_inc{0.1f};
    constexpr float step_size{0.2f};
    constexpr int move_steps{3};

    neighbors.clear();
    State search_state;
    for (float sa = steer_start; sa < steer_end; sa += steer_inc) {
        // forward
        search_state = current;
        kinematic_model(search_state, sa, 0);
        for (int i = 0; i < move_steps; ++i) {
            kinematic_model(search_state, sa, step_size * i);
            neighbors.push_back(search_state);
        }

        // backward
        search_state = current;
        kinematic_model(search_state, sa, 0);
        for (int i = 0; i < move_steps; ++i) {
            kinematic_model(search_state, sa, -step_size * i);
            neighbors.push_back(search_state);
        }
    }
}

void HybridAStar::kinematic_model(State& state, float steer_angle, float moves)
{
    constexpr float dt{0.02f};          // sec.
    constexpr float wheel_base{2.8f};   // meter

    const float r{wheel_base / std::tan(steer_angle)};
    const float yaw_gain{moves / r};
    state.heading += yaw_gain;
    const float x_gain{moves * std::cos(state.heading)};
    const float y_gain{moves * std::sin(state.heading)};
    state.x += x_gain;
    state.y += y_gain;
}

int main()
{
    auto viz{rviz::Viz::instance()};

    State init{.x=0.0f, .y=0.0f, .heading=0.0f};
    State goal{.x=10.0f, .y=10.0f, .heading=-1.0f};
    HybridAStar has{init, goal};
    has.search();

    while (!viz->closed()) {
        viz->render();
    }
    return 0;
}
