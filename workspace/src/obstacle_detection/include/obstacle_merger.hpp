#ifndef OBSTACLE_MERGER_HPP
#define OBSTACLE_MERGER_HPP

#include "contracts/detecting_sensor.hpp"
#include "contracts/image_obstacle.hpp"
#include "contracts/obstacle.hpp"
#include "contracts/obstacle_type.hpp"
#include "contracts/perception_params.hpp"

namespace magellan
{
    namespace obstacle_detection
    {
        class ObstacleMerger
        {
            public:
                ObstacleMerger(const PerceptionParams &params);

                bool MergeObstacles(
                        const std::vector<std::vector<Obstacle>> &detected_obstacles,
                        std::vector<Obstacle> &merged_obstacles);

                bool SetParameters(const PerceptionParams &params);

            private:
                PerceptionParams _params;

                // TODO: some data structure to keep track of the seen obstacles
        };
    }
}


#endif
