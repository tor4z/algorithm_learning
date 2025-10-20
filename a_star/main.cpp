#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>

#define RS_PATH_IMPLEMENTATION
#include "rspath.h"

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


#define ROBOT_TURN_RADIUS 3.0f

template<typename T>
class PriorityQueue
{
public:
    PriorityQueue() : cmp_([](const T& a, const T& b) { return a < b ? -1 : 1; }) {}
    // explicit PriorityQueue(std::function<int(const T&, const T&)> cmp)
    //     : cmp_(cmp) {}
    ~PriorityQueue() = default;
    void enque(const T& v);
    void enque(T&& v);
    void deque();

    inline size_t size() { return data_.size(); }
    inline const T& top() const { return data_.at(0); }
    inline bool empty() const { return data_.empty(); }
private:
    void heaped_up(size_t i);
    void heaped_down(size_t i);
    std::function<int(const T&, const T&)> cmp_;
    std::vector<T> data_;
}; // class PriorityQueue


struct State
{
    float x;
    float y;
    float heading;
}; // struct State

template<typename T>
inline T pow2(T v) { return v * v; }

template<typename T>
inline T pow3(T v) { return v * v * v; }

inline float euclidean_dist(const State& a, const State& b)
{
    return std::sqrt(pow2(a.x - b.x) + pow2(a.y - b.y));
}

template<typename T>
void PriorityQueue<T>::enque(const T& v)
{
    data_.push_back(v);
    heaped_up(data_.size() - 1);
}

template<typename T>
void PriorityQueue<T>::enque(T&& v)
{
    data_.push_back(std::move(v));
    heaped_up(data_.size() - 1);
}

template<typename T>
void PriorityQueue<T>::deque()
{
    if (data_.empty()) return;
    if (data_.size() == 1) {
        data_.pop_back();
        return;
    }

    data_.at(0) = data_.at(data_.size() - 1);
    data_.pop_back();
    heaped_down(0);
}

template<typename T>
void PriorityQueue<T>::heaped_up(size_t i)
{
    while (i > 0) {
        const auto parent{(i - 1) / 2};
        if (cmp_(data_.at(parent), data_.at(i)) > 0) {
            std::swap(data_.at(parent), data_.at(i));
        }
        i = parent;
    }
}

template<typename T>
void PriorityQueue<T>::heaped_down(size_t i)
{
    const auto que_size{data_.size()};
    while (i < que_size) {
        const auto left_child{i * 2 + 1};
        const auto right_child{i * 2 + 2};
        if (left_child >= que_size) break;
        size_t min_idx{left_child};
        if (right_child < que_size && cmp_(data_.at(min_idx), data_.at(right_child)) > 0) {
            min_idx = right_child;
        }

        if (cmp_(data_.at(min_idx), data_.at(i)) < 0) {
            std::swap(data_.at(min_idx), data_.at(i));
            i = min_idx;
        } else {
            break;
        }
    }
}

class HybridAStar
{
public:
    HybridAStar(const State& init, const State& goal);
    ~HybridAStar() = default;
    bool search();
private:
    void find_neighbors(const State& current, std::vector<State>& neighbors);
    float neighbor_cost(const State& neighbor);
    void kinematic_model(State& state, float steer_angle, float moves);

    struct StateCost {
        State state;
        float cost;
        bool operator<(const StateCost& other) const { return cost < other.cost ? true : false; }
    };

    PriorityQueue<StateCost> pq;
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
    auto viz{rviz::Viz::instance()};

    pq.enque({.state=init_, .cost=0.0f});
    while (!pq.empty()) {
        const auto current{pq.top().state};
        pq.deque();
        viz->draw_trj2d_point("test/trj", current.x, current.y);
        if (euclidean_dist(current, goal_) < short_distance) {
            break;
        }

        find_neighbors(current, neighbors);
        if (neighbors.empty()) {
            continue;
        }

        for (size_t i = 0; i < neighbors.size(); ++i) {
            const auto& neighbor{neighbors.at(i)};
            const auto this_cost{neighbor_cost(neighbor)};
            pq.enque({.state=neighbor, .cost=this_cost});
        }
    }

    return true;
}

float HybridAStar::neighbor_cost(const State& neighbor)
{
    const float dist{euclidean_dist(goal_, neighbor)};
    if (dist > 10.0f) {
        return (std::abs(neighbor.x - goal_.x) + std::abs(neighbor.y - goal_.y));
    } else {
        RsPath rs_path;
        const float dx{goal_.x - neighbor.x};
        const float dy{goal_.y - neighbor.y};
        const float cos_yaw{std::cos(neighbor.heading)};
        const float sin_yaw{std::sin(neighbor.heading)};
        const float x{cos_yaw * dx + sin_yaw * dy};
        const float y{-sin_yaw * dx + cos_yaw * dy};
        const float phi{goal_.heading - neighbor.heading};
        rs_find_from_all_path(x / ROBOT_TURN_RADIUS, y / ROBOT_TURN_RADIUS, phi, &rs_path);
        return rs_path.length * ROBOT_TURN_RADIUS;
    }
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
    State goal{.x=10.0f, .y=10.0f, .heading=M_PI_2};
    HybridAStar has{init, goal};
    has.search();

    while (!viz->closed()) {
        viz->render();
    }
    return 0;
}
