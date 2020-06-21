#ifndef PLANNER_TYPES_HPP
#define PLANNER_TYPES_HPP

typedef struct OccupancyGridSquare
{
    int x;
    int y;

    OccupancyGridSquare()
        : x(0), y(0) {};
    OccupancyGridSquare(int x, int y)
        : x(x), y(y) {};

} OccupancyGridSquare_t;

#endif
