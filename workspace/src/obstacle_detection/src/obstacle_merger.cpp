#include "obstacle_merger.hpp"

namespace mo = magellan::obstacle_detection;

mo::ObstacleMerger::ObstacleMerger(const PerceptionParams &params) :
    _params(params)
{

}

bool mo::ObstacleMerger::MergeObstacles(
        const std::vector<std::vector<mo::Obstacle>> &detected_obstacles,
        std::vector<mo::Obstacle> &merged_obstacles)
{
    return true;
}

bool mo::ObstacleMerger::SetParameters(const PerceptionParams &params)
{
    return true;
}
