#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <vector>

class Obstacle
{
    public:
        Obstacle(bool is_cone, std::vector<unsigned int> &grid_coords)
            : is_cone(is_cone), grid_coords(std::move(grid_coords)) {};

        bool get_is_cone() const { return this->is_cone; }
        const std::vector<unsigned int>& get_grid_coords() const { return this->grid_coords; }

    private:
        bool is_cone;
        std::vector<unsigned int> grid_coords;
};

#endif
