#include <cstdio>
#include <vector>
#include "rspath.h"
#include "turtle.h"
#include "gnuplot.hpp"


int main()
{
    Turtle bob = {0};
    RsPath path;

    rs_path_lprmlp(-3.0, 2.0, 0.0, &path);
    turtle_arc(&bob, 1.0f, path.pattern_val[0]);
    turtle_arc(&bob, -1.0f, path.pattern_val[1] * -1);
    turtle_arc(&bob, 1.0f, path.pattern_val[2]);

    printf("%f, %f, %f\n", path.pattern_val[0], path.pattern_val[1], path.pattern_val[2]);
    std::vector<float> xs;
    std::vector<float> ys;
    for (int i = 0; i < bob.trj.size; ++i) {
        xs.push_back(bob.trj.items[i].x);
        ys.push_back(bob.trj.items[i].y);
        printf("%f %f\n", bob.trj.items[i].x, bob.trj.items[i].y);
    }

    gp::Plotter p;
    p.cmd("set size ratio -1");
    p.line(xs.data(), ys.data(), xs.size());
    return 0;
}
