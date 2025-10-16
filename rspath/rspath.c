#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>


#include "turtle.h"


/*
 * Reeds, J.A., & Shepp, L.A. (1990). OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS.
 * Pacific Journal of Mathematics, 145, 367-393.
 *
 */

#define RS_NUM_PATTERNS 5

typedef enum {
    SP_L = 0,
    SP_R,
    SP_S
} SegPattern;

typedef struct {
    int num_segs;
    float length;
    float pattern_val[RS_NUM_PATTERNS];
    SegPattern patterns[RS_NUM_PATTERNS];
} RsPath;

static inline float pow2f(float x)
{
    return x * x;
}

static inline float pow3f(float x)
{
    return x * x * x;
}

static inline float absf(float x)
{
    return x > 0.0f ? x : -x;
}

static inline float norm_2pif(float x)
{
    static const float DB_PIf = 2.0f * M_PI;

    while (x > DB_PIf) { x -= DB_PIf; }
    while (x < -DB_PIf) { x += DB_PIf; }
    return x;
}

bool to_polar(float x, float y, float* r, float* theta)
{
    if (!r || !theta) return false;
    *r = sqrtf(pow2f(x) + pow2f(y));
    *theta = atan2f(y, x);
    return true;
}

// pp 190. eq. 8.1
bool path_csc(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u;
    float t;
    float v;
    to_polar(x - sin(phi), y - 1.0f + cos(phi) , &u, &t);
    v = norm_2pif(phi - t);
    path->length = absf(u) + absf(t) + absf(v);
    path->num_segs = 3;
    path->patterns[0] = SP_L;
    path->patterns[1] = SP_L;
    path->patterns[2] = SP_L;
    path->pattern_val[0] = t;
    path->pattern_val[1] = u;
    path->pattern_val[2] = v;
    return true;
}

void f(Turtle* t)
{
    (&(t->trj))->items = malloc(100);
}


int main()
{
    Turtle turtle = {0};
    turtle_forward(&turtle, 2.0f);
    printf("%d\n", turtle.trj.size);
    return 0;
}
