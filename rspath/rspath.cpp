#include <cstdio>
#include <vector>
#include "rspath.h"
#include "turtle.h"
#include "gnuplot.hpp"


int main()
{
    Turtle bob = {0};
    RsPath path;

    rs_find_from_all_path(-3.0, 2.0, 0.0, &path);
    printf("min path length: %f\n", path.length);
    return 0;
    turtle_arc(&bob, 1.0f, path.segs[0].val);
    turtle_arc(&bob, -1.0f, path.segs[1].val * -1);
    turtle_arc(&bob, 1.0f, path.segs[2].val);

    printf("%f, %f, %f\n", path.segs[0].val, path.segs[1].val, path.segs[2].val);
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
