#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <ostream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

#define NUM_LANES 3
#define TARGET_SPD 30.0f // mps
#define STEP_SIZE 0.02f

enum Behavior {
    BH_UNKNOWN = 0,
    BH_CRUISE,
    BH_FOLLOW,
    BH_AEB,
    BH_COMFORT_STOP,
    BH_CHANGE_LEFT,
    BH_CHANGE_RIGHT,
}; // enum Behavior

inline const std::string& stringify(Behavior b)
{
    static std::string bh_unknown{"UNKNOWN"};
    static std::string bh_cruise{"CRUISE"};
    static std::string bh_follow{"FOLLOW"};
    static std::string bh_aeb{"AEB"};
    static std::string bh_comfort_stop{"COMFORT_STOP"};
    static std::string bh_change_left{"CHANGE_LEFT"};
    static std::string bh_change_right{"CHANGE_RIGHT"};

    switch (b) {
    case BH_UNKNOWN: return bh_unknown;
    case BH_CRUISE: return bh_cruise;
    case BH_FOLLOW: return bh_follow;
    case BH_AEB: return bh_aeb;
    case BH_COMFORT_STOP: return bh_comfort_stop;
    case BH_CHANGE_LEFT: return bh_change_left;
    case BH_CHANGE_RIGHT: return bh_change_right;
    }
    return bh_unknown;
}

template<typename T>
inline T rand_ab(T a, T b)
{
    assert(b > a);
    float rv{static_cast<float>(rand()) / static_cast<float>(RAND_MAX)};
    return rv * (b - a) + a;
}

inline bool prob(float p)
{
    return rand_ab(0.0f, 1.0f) < p;
}

template<typename T>
inline T abs(T v) { return v > 0 ? v : -v; }

inline int gen_id()
{
    static int cnt{0};
    return (cnt++ % 100000);
}

struct Vec2
{
    float x;
    float y;
}; // struct Vec2

struct Control
{
    float accel;
}; // struct Control

std::ostream& operator<<(std::ostream& os, const Vec2& v)
{
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

struct BehaviorInfo
{
    int lane_id;
    int lead_id;
    int lead_lane_id;
    float lead_vel;
    float lead_accel;
    Vec2 lead_position;
    Behavior behavior;
}; // struct BehaviorInfo

struct VehModel
{
    int id;
    int lane_id;
    float heading;
    float accel;
    float vel;
    bool controled;
    Vec2 position;

    void step(float dt);
    void act(const Control& control);
    void behavior_plan(BehaviorInfo& bi);
    void vel_plan(const BehaviorInfo& bi, Control& control);
}; // struct VehModel

std::vector<VehModel> participants;
VehModel ego;

void VehModel::step(float dt)
{
    vel += accel * dt;
    position.x += vel * dt;
    if (!controled) {
        if (vel > 10.0f) {
            accel += rand_ab(-0.1f, 0.1f);
        } else {
            accel += 0.1f;
        }

        bool ch_lane{prob(0.02f) && abs(position.x - ego.position.x) > 5.0f};
        if (ch_lane) {
            lane_id = (lane_id + 1) % NUM_LANES;
            std::cout << "Change lane, ego: " << ego.position << ", par: " << position << ", "
                      << "lead vel: " << vel << "\n";
        }
    }
}

bool check_collision()
{
    for (const auto& p : participants) {
        if (p.lane_id == ego.lane_id &&
            abs(p.position.x - ego.position.x) < 5.0f &&
            p.position.x > ego.position.x &&
            p.vel > ego.vel) {
            std::cout << "== EGO ==\n"
                      << "  position: " << ego.position << "\n"
                      << "  lane id: " << ego.lane_id << "\n"
                      << "  vel: " << ego.vel << "\n"
                      << "== TARGET ==\n"
                      << "  position: " << p.position << "\n"
                      << "  lane id: " << p.lane_id << "\n"
                      << "  id: " << p.id << "\n"
                      << "  vel: " << p.vel << "\n";
            return true;
        }
    }
    return false;
}

void VehModel::act(const Control& control)
{
    accel = control.accel;
}

void VehModel::behavior_plan(BehaviorInfo& bi)
{
    bi.behavior = BH_CRUISE;
    bi.lead_id = -1;

    if (!participants.empty()) {
        float nearest_follow{INFINITY};
        for (const auto& p : participants) {
            if (p.lane_id == ego.lane_id) {
                if (p.position.x >= ego.position.x && p.position.x < nearest_follow &&
                    abs(p.position.x - ego.position.x) < 80.0f) {
                    nearest_follow = p.position.x;
                    bi.behavior = BH_FOLLOW;
                    bi.lead_accel = p.accel;
                    bi.lead_id = p.id;
                    bi.lead_lane_id = p.lane_id;
                    bi.lead_position = p.position;
                    bi.lead_vel = p.vel;
                }
            }
        }
    }
}

void VehModel::vel_plan(const BehaviorInfo& bi, Control& control)
{
    if (vel < 0.001) {
        accel = 0.0f;
    }
    switch (bi.behavior) {
    case BH_FOLLOW:
        if (vel > bi.lead_vel) {
            accel = -5.0f;
        } else {
            accel = 0.0f;
        }
        break;
    case BH_AEB:
        break;
    case BH_COMFORT_STOP:
        break;
    case BH_CHANGE_LEFT:
        break;
    case BH_CHANGE_RIGHT:
        break;
    case BH_CRUISE:
    case BH_UNKNOWN:
    default:
        break;
    }
}

void report_behavior(const BehaviorInfo& bi)
{
    std::cout << "The plan behavior: " << stringify(bi.behavior);
    if (bi.behavior == BH_FOLLOW) {
        std::cout
            << ", lead: " << bi.lead_position
            << ", ego: " << ego.position
            << ", lead id: " << bi.lead_id
            << ", vel: " << ego.vel
            << ", lead vel: " << bi.lead_vel
            << ", lead lane id: " << bi.lead_lane_id;
    }
    std::cout << "\n";
}

void report_scene()
{
    std::cout << "== EGO ==\n"
        << " position: " << ego.position << "\n"
        << " vel: " << ego.vel << "\n"
        << " accel: " << ego.accel << "\n";
    std::cout << "== PARTICIPANTS ==\n";
    for (const auto& p: participants) {
        std::cout
            << " position: " << p.position << "\n"
            << " vel: " << p.vel << "\n"
            << " accel: " << p.accel << "\n"
            << "---\n";
    }
}

void step(float dt)
{
    ego.step(STEP_SIZE);
    for (auto& p: participants) {
        p.step(STEP_SIZE);
    }
}

int main()
{
    srand(time(NULL));

    ego.lane_id = 1;
    ego.id = gen_id();
    ego.controled = true;
    ego.vel = 20.0f;

    VehModel p{};
    for (int i = 0; i < 10; ++i) {
        p.id = gen_id();
        p.lane_id = rand_ab(0, NUM_LANES - 1);
        p.position.x = rand_ab(-10.0f, 100.0f);
        p.accel = 0.0f;
        p.vel = rand_ab(10.0f, 40.0f);
        if (p.lane_id == ego.lane_id &&
            abs(p.position.x - ego.position.x) < 5.0f) {
            continue;
            --i;
        }
        participants.push_back(p);
    }

    BehaviorInfo bi;
    Control control;
    while (true) {
        std::this_thread::sleep_for(20ms);
        step(0.02f);
        ego.behavior_plan(bi);
        ego.vel_plan(bi, control);
        ego.act(control);

        report_behavior(bi);
        if (check_collision()) {
            std::cerr << "Collision!!\n";
            return 1;
        }

        static int cnt{0};
        if (cnt++ % 100 == 0)
            report_scene();
    }
    return 0;
}
