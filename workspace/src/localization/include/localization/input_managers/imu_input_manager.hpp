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
                const vector3r_t& get_global_acceleration() const;
                const quaternionr_t& get_global_heading() const;
                const quaternionr_t& get_global_angular_velocity() const;
                const Eigen::Matrix<real_t, state_dimension, state_dimension>& get_measurement_covariance() const;
                bool data_ready();

                void add_imu_message(magellan_messages::MsgMagellanImu &msg);
                void recompute(const GlobalPose &global_pose);

            private:
                vector3r_t last_global_acceleration;
                quaternionr_t last_global_heading;
                quaternionr_t last_global_angular_velocity;

                Eigen::Matrix<real_t, state_dimension, state_dimension> covariance_matrix;
        };
    }
}

#endif