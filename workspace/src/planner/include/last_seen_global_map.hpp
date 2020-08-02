#ifndef LAST_SEEN_GLOBAL_MAP_HPP
#define LAST_SEEN_GLOBAL_MAP_HPP

#include <algorithm>
#include <math.h>

#include <geometry_msgs/Point.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include "geometry_utils.hpp"
#include "global_map.hpp"
#include "planner_types.hpp"

class LastSeenGlobalMap : public GlobalMap
{
    public:
        LastSeenGlobalMap(
            const geometry_msgs::Pose &goal_pose,
            float x_border,
            float y_border,
            float resolution_in_m);

        virtual void update_map(
            const magellan_messages::MsgZedPose &pose,
            const magellan_messages::MsgMagellanOccupancyGrid &obstacles) override;

        virtual const magellan_messages::MsgMagellanOccupancyGrid& get_map() const override;

        virtual float get_resolution() const override;

        virtual OccupancyGridSquare_t real_to_grid(const geometry_msgs::Point &real_world_point) const override;

        virtual geometry_msgs::Point grid_to_real(const OccupancyGridSquare_t &square) const override;

    private:
        magellan_messages::MsgMagellanOccupancyGrid grid;
        float min_world_x;
        float min_world_y;
        float max_world_x;
        float max_world_y;
        float resolution_in_m;
        int num_x_squares;
        int num_y_squares;
};

#endif
