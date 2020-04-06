#ifndef IMU_INPUT_MANAGER_HPP
#define IMU_INPUT_MANAGER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>

#include <magellan_messages/MsgMagellanImu.h>

#include "contracts/global_pose.hpp"
#include "definitions.hpp"

namespace magellan
{
    namespace localization
    {
        class ImuInputManager
        {
            public:
                ImuInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &measurement_covariance);
                const vector3r_t& get_local_acceleration() const;
                const vector3r_t& get_global_rotation() const;
                const vector3r_t& get_local_angular_velocity() const;
                const Eigen::Matrix<real_t, 3, 3> get_local_acceleration_covariance();
                const Eigen::Matrix<real_t, 3, 3> get_global_heading_covariance();
                const Eigen::Matrix<real_t, 3, 3> get_local_angular_velocity_covariance();
                int num_unread_messages();

                void add_imu_message(magellan_messages::MsgMagellanImu &msg);
                void recompute(const GlobalPose &global_pose);

            private:
                vector3r_t last_local_acceleration;
                vector3r_t last_global_heading;
                vector3r_t last_local_angular_velocity;

                Eigen::Matrix<real_t, 3, 3> last_local_acceleration_covaraince;
                Eigen::Matrix<real_t, 3, 3> last_global_heading_covariance;
                Eigen::Matrix<real_t, 3, 3> last_local_angular_velocity_covariance;
        };
    }
}

#endif
