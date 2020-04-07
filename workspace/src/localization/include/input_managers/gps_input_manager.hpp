#ifndef GPS_INPUT_MANAGER_HPP
#define GPS_INPUT_MANAGER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geodesy/utm.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "contracts/global_pose.hpp"
#include "definitions.hpp"

namespace magellan
{
    namespace localization
    {
        class GpsInputManager
        {
            public:
                GpsInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&measurement_covariance);
                ~GpsInputManager();
                const vector3r_t& get_global_position() const;
                const Eigen::Matrix<real_t, 3, 3> get_global_position_covariance() const;
                int num_unread_messages();

                void add_gps_message(const sensor_msgs::NavSatFix::ConstPtr &msg);
                void recompute(const GlobalPose &global_pose);

            private:
                int unread_message_count = 0;
                pthread_mutex_t processing_mutex;

                vector3r_t last_global_position;
                Eigen::Matrix<real_t, 3, 3> last_global_position_covariance;

                real_t latitude_tableau;
                real_t longitude_tableau;
                real_t altitude_tableau;
                Eigen::Matrix<real_t, 3, 3> global_position_covariance;
        };
    }
}


#endif
