#ifndef IMU_INPUT_MANAGER_HPP
#define IMU_INPUT_MANAGER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pthread.h>
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
                ImuInputManager(
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&local_acceleration_covariance,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&global_heading_covariance,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&local_angular_velocity_covariance);
                ~ImuInputManager();
                const vector3r_t& get_local_acceleration() const;
                const vector3r_t& get_global_rotation() const;
                const vector3r_t& get_local_angular_velocity() const;
                const Eigen::Matrix<real_t, 3, 3>& get_local_acceleration_covariance() const;
                const Eigen::Matrix<real_t, 3, 3>& get_global_heading_covariance() const;
                const Eigen::Matrix<real_t, 3, 3>& get_local_angular_velocity_covariance() const;
                int num_unread_messages();

                void add_imu_message(const magellan_messages::MsgMagellanImu::ConstPtr &msg);
                void recompute(const GlobalPose &global_pose);

            private:
                vector3r_t last_local_acceleration;
                vector3r_t last_global_heading;
                vector3r_t last_local_angular_velocity;

                vector3r_t local_acceleration_tableau;
                vector3r_t global_heading_tableau;
                vector3r_t local_angular_velocity_tableau;

                int unread_message_count = 0;
                pthread_mutex_t processing_mutex;

                Eigen::Matrix<real_t, 3, 3> last_local_acceleration_covariance;
                Eigen::Matrix<real_t, 3, 3> last_global_heading_covariance;
                Eigen::Matrix<real_t, 3, 3> last_local_angular_velocity_covariance;

                Eigen::Matrix<real_t, 3, 3> local_acceleration_covariance;
                Eigen::Matrix<real_t, 3, 3> global_heading_covariance;
                Eigen::Matrix<real_t, 3, 3> local_angular_velocity_covariance;
        };
    }
}

#endif
