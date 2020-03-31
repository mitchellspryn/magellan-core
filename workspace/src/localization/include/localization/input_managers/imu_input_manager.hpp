#ifndef IMU_INPUT_MANAGER_HPP
#define IMU_INPUT_MANAGER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>

#include <magellan_messages/MsgMagellanImu.h>

#include "constants.hpp"
#include "contracts/global_pose.hpp"

namespace magellan
{
    namespace localization
    {
        class ImuInputManager
        {
            public:
                ImuInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &measurement_covariance);
                ~ImuInputManager();

                void add_imu_message(magellan_messages::MsgMagellanImu &msg);
                void recompute(const GlobalPose const &global_pose);
                bool data_ready();

                const vector3r_t& get_global_acceleration() const;
                const quaternionr_t& get_global_heading() const;
                const quaternionr_t& get_global_angular_velocity() const;

                const Eigen::Matrix<real_t, state_dimension, state_dimension> get_measurement_covariance() const;

            private:
                vector3r_t last_global_acceleration;
                quaternionr_t last_global_heading;
                quaternionr_t last_global_angular_velocity;

                Eigen::Matrix<real_t, state_dimension, state_dimension> covariance_matrix;
        };
    }
}

#endif
