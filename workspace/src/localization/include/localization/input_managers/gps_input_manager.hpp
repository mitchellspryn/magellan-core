#ifndef GPS_INPUT_MANAGER_HPP
#define GPS_INPUT_MANAGER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "constants.hpp"
#include "contracts/global_pose.hpp"

namespace magellan
{
    namespace localization
    {
        class GpsInputManager
        {
            public:
                GpsInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &measurement_covariance);

                ~GpsInputManager();

                void add_gps_message(sensor_msgs::NavSatFix &msg);
                void recompute(const GlobalPose const &global_pose);
                bool data_ready();

                const vector3r_t& get_global_position() const;

                const Eigen::Matrix<real_t, state_dimension, state_dimension> get_measurement_covariance() const;

            private:
                vector3r_t last_global_position;
                Eigen::Matrix<real_t, state_dimension, state_dimension> covariance_matrix;
        }
    }
}


#endif
