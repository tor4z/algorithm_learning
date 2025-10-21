#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"

#define WHEEL_BASE  2.8f                        // meter
#define MAP_X_MIN   0.0f                        // meter
#define MAP_X_MAX   200.0f                      // meter
#define MAP_X_SIZE  (MAP_X_MAX - MAP_X_MIN)     // meter
#define MAP_Y_MIN   0.0f                        // meter
#define MAP_Y_MAX   200.0f                      // meter
#define MAP_Y_SIZE  (MAP_Y_MAX - MAP_Y_MIN)     // meter


class RRT
{
public:
    struct State
    {
        float x;
        float y;
        float heading;
        int parent;
    }; // struct State

    struct Graph
    {
        Graph();
        ~Graph() = default;
        bool add_init_node(const State& point);
        bool add_edges(int src_idx, const State& b);
        int find_nearest(const State& ref);

        inline State& last_node() { return vertices_.back(); }
        inline State& node(int idx) { return vertices_.at(idx); }
        inline bool empty() { return vertices_.empty(); }
    private:
        std::vector<State> vertices_;
    }; // struct Graph

    explicit RRT(const State& goal);
    ~RRT() = default;
    bool search(const State& init, int max_iter, float short_distance);
    bool extract_path(std::vector<State>& path);
private:
    State goal_;
    Graph g_;

    bool random_point(State& point);
    bool steer(const State& from, const State& to, State& new_state);
}; // class RRT

inline float rand_01()
{
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

template<typename T>
inline T pow2(T v) { return v * v; }

template<typename T>
inline T pow3(T v) { return v * v * v; }

template<typename T>
inline float euclidean_dist(const T& a, const T& b)
{
    return std::sqrt(pow2(a.x - b.x) + pow2(a.y - b.y));
}

RRT::RRT(const State& goal)
    : goal_(goal)
{}

bool RRT::random_point(State& point)
{
    point.x = rand_01() * MAP_X_SIZE + MAP_X_MIN;
    point.y = rand_01() * MAP_Y_SIZE + MAP_Y_MIN;
    return false;
}

bool RRT::search(const State& init, int max_iter, float short_distance)
{
    State rand_point;
    State nearest_node;
    State new_node;
    float nearest_node_to_goal{1.0e6f};

    auto viz{rviz::Viz::instance()};
    g_.add_init_node(init);
    for (int i = 0; i < max_iter; ++i) {
        random_point(rand_point);
        const int nearest_idx{g_.find_nearest(rand_point)};
        if (nearest_idx < 0) continue;

        nearest_node = g_.node(nearest_idx);
        if (!steer(nearest_node, rand_point, new_node)) continue;

        g_.add_edges(nearest_idx, new_node);
        nearest_node_to_goal = std::min(nearest_node_to_goal, euclidean_dist(new_node, goal_));
        const rviz::Point2f seg_start{.x=nearest_node.x, .y=nearest_node.y};
        const rviz::Point2f seg_end{.x=new_node.x, .y=new_node.y};
        viz->draw_line_segment_("test/segs", seg_start, seg_end);
        if (nearest_node_to_goal < short_distance) {
            break;
        }
    }

    if (nearest_node_to_goal < short_distance) {
        return true;
    }
    return false;
}

bool RRT::steer(const State& from, const State& to, State& new_state)
{
    constexpr float steer_start{-0.5f}; // rad
    constexpr float steer_end{0.5f};    // rad
    constexpr float steer_inc{0.1f};    // rad
    constexpr float step_size{0.5f};    // meter

    float min_dist{1.0e6f};
    State tmp_state;
    for (float s = steer_start; s < steer_end; s += steer_inc) {
        const float r{WHEEL_BASE / std::tan(s)};
        const float yaw_gain{step_size / r};
        tmp_state.heading = from.heading + yaw_gain;
        tmp_state.x = from.x + step_size * std::cos(tmp_state.heading);
        tmp_state.y = from.y + step_size * std::sin(tmp_state.heading);
        const float this_dist{euclidean_dist(tmp_state, to)};
        if (this_dist < min_dist) {
            min_dist = this_dist;
            new_state = tmp_state;
        }
    }

    return true;
}

bool RRT::extract_path(std::vector<State>& path)
{
    if (g_.empty()) return false;

    State curr{g_.last_node()};
    path.push_back(curr);
    while (curr.parent >= 0) {
        curr = g_.node(curr.parent);
        path.push_back(curr);
    }

    return true;
}

RRT::Graph::Graph()
{}

bool RRT::Graph::add_init_node(const State& point)
{
    vertices_.push_back(point);
    vertices_.back().parent = -1;
    return true;
}

bool RRT::Graph::add_edges(int src, const State& b)
{
    vertices_.push_back(b);
    vertices_.back().parent = src;
    return true;
}

int RRT::Graph::find_nearest(const State& ref)
{
    if (vertices_.empty()) {
        return false;
    }

    int result{-1};
    float min_dist{1.0e6f};
    for (size_t i = 0; i < vertices_.size(); ++i) {
        const auto& this_node{vertices_.at(i)};
        const auto this_dist{euclidean_dist(this_node, ref)};
        if (this_dist < min_dist) {
            min_dist = this_dist;
            result = i;
        }
    }

    return result;
}

int main()
{
    RRT::State init{.x=0.0f, .y=0.0f, .heading=0.0f};
    RRT::State goal{.x=70.0f, .y=20.0f, .heading=M_PI_2};

    RRT rrt{goal};
    std::vector<RRT::State> path;

    if (!rrt.search(init, 10000, 10.0f)) {
        std::cerr << "Search path with RRT failed\n";
        return 1;
    }

    if (!rrt.extract_path(path)) {
        std::cerr << "Extract RRT path failed\n";
        return 1;
    }

    std::cout << "path length: " << path.size() << "\n";
    rviz::Point2f p;
    auto viz{rviz::Viz::instance()};
    for (const auto& it : path) {
        p.x = it.x;
        p.y = it.y;
        viz->draw_trj2d_point_("test/path", p);
    }
    RVIZ_RENDER_UNTIL_CLOSED();
    return 0;
}
