#include "filters/linear_kalman_filter.hpp"

namespace magellan
{
    namespace localization
    {
        LinearKalmanFilter::LinearKalmanFilter(std::string parameter_path)
        {
            this->initialize_parameters_from_file(parameter_path);
            this->initialize_internal_state();
        }

        void LinearKalmanFilter::accept_imu_message(magellan_messages::MsgMagellanImu &msg)
        {
            this->imu_input_manager->add_imu_message(msg);
        }

        void LinearKalmanFilter::accept_gps_message(sensor_msgs::NavSatFix &msg)
        {
            this->gps_input_manager->add_gps_message(msg);
        }

        void LinearKalmanFilter::accept_drive_message(magellan_messages::MsgMagellanDrive &msg)
        {
            this->drive_input_manager->add_drive_message(msg);
        }

        void LinearKalmanFilter::update_global_pose(ros::Duration duration)
        {
            bool imu_ready = false;
            bool gps_ready = false;

            if (this->imu_input_manager->num_unread_messages() > 0)
            {
                imu_ready = true;
                this->imu_input_manager->recompute(this->global_pose);
            }

            if (this->gps_input_manager->num_unread_messages() > 0)
            {
                gps_ready = true;
                this->gps_input_manager->recompute(this->global_pose);
            }

            this->drive_input_manager->recompute(this->global_pose);

            Eigen::Matrix<real_t, 3, 3> local_to_global_mat = this->roll_pitch_yaw_to_rotation_matrix(this->global_pose.global_rotation);

            // TODO: What is the performance overhead of assigning all of these here?
            Eigen::Matrix<real_t, state_dimension, state_dimension> A = Eigen::Matrix<real_t, state_dimension, state_dimension>::Identity();
            Eigen::Matrix<real_t, state_dimension, 1> Bx = this->drive_input_manager->get_control_matrix();
            Eigen::Matrix<real_t, state_dimension, state_dimension> Q = this->rotate_covariance_matrix(local_to_global_mat, this->drive_input_manager->get_covariance_matrix());
            Eigen::Matrix<real_t, state_dimension, state_dimension> R = Eigen::Matrix<real_t, state_dimension, state_dimension>::Zero();
            Eigen::Matrix<real_t, state_dimension, state_dimension> H = Eigen::Matrix<real_t, state_dimension, state_dimension>::Zero();
            Eigen::Matrix<real_t, state_dimension, 1> X_predicted = Eigen::Matrix<real_t, state_dimension, 1>::Zero();
            Eigen::Matrix<real_t, state_dimension, state_dimension> P_predicted = Eigen::Matrix<real_t, state_dimension, state_dimension>::Zero();
            Eigen::Matrix<real_t, state_dimension, state_dimension> K = Eigen::Matrix<real_t, state_dimension, state_dimension>::Zero();
            Eigen::Matrix<real_t, state_dimension, 1> Z = Eigen::Matrix<real_t, state_dimension, 1>::Zero();

            if (imu_ready)
            {
                R.block<3, 3>(6, 6) = this->rotate_covariance_matrix(local_to_global_mat, this->imu_input_manager->get_local_acceleration_covariance());
                R.block<3, 3>(9, 9) = this->imu_input_manager->get_global_heading_covariance();
                R.block<3, 3>(12, 12) = this->rotate_covariance_matrix(local_to_global_mat, this->imu_input_manager->get_local_angular_velocity_covariance());

                Z.block<3, 1>(6, 0) = this->rotate_vector(local_to_global_mat, this->imu_input_manager->get_local_acceleration());
                Z.block<3, 1>(9, 0) = this->imu_input_manager->get_global_rotation();
                Z.block<3, 1>(12, 0) = this->rotate_vector(local_to_global_mat, this->imu_input_manager->get_local_angular_velocity());

                H.block<9, 9>(6, 6) = Eigen::Matrix<real_t, 9, 9>::Identity();
            }

            if (gps_ready)
            {
                R.block<3, 3>(0, 0) = this->gps_input_manager->get_global_position_covariance();

                Z.block<3, 1>(0, 0) = this->gps_input_manager->get_global_position();

                H.block<3, 3>(0, 0) = Eigen::Matrix<real_t, 3, 3>::Identity();
            }

            // Set up the kinematics to be simple physics
            // x_t+1 = x + dt*x_dot + (dt^2/2)*x_ddot
            float dt = static_cast<float>(duration.toSec());
            float accel_factor = (dt*dt) / 2.0f;
            for (int i = 0; i < 6; i++)
            {
                A(i, i+3) = dt;
            }
            for (int i = 0; i < 3; i++)
            {
                A(i, i+6) = accel_factor;
            }
            for (int i = 0; i < 3; i++)
            {
                A(i+9, i+12) = dt;
            }

            // Perform an iteration of kalman
            X_predicted = (A*this->state) + Bx;
            P_predicted = (A*this->state_covariance*(A.transpose())) + Q;

            Eigen::Matrix<real_t, state_dimension, state_dimension> PH_t = P_predicted*(H.transpose());
            K = PH_t * (((H*PH_t) + R).inverse());

            this->state = X_predicted + K*(Z - (H*X_predicted));
            this->state_covariance = (Eigen::Matrix<real_t, state_dimension, state_dimension>::Identity() - (K*H))*P_predicted;

            // Update global state
            this->global_pose.global_position = this->state.block<3, 1>(0, 0);
            this->global_pose.global_velocity = this->state.block<3, 1>(3, 0);
            this->global_pose.global_acceleration = this->state.block<3, 1>(6, 0);
            this->global_pose.global_rotation = this->state.block<3, 1>(9, 0);
            this->global_pose.global_angular_velocity = this->state.block<3, 1>(12, 0);

            this->global_pose.global_position_covariance = this->state_covariance.block<3, 3>(0, 0);
            this->global_pose.global_velocity_covariance = this->state_covariance.block<3, 3>(3, 3);
            this->global_pose.global_acceleration_covariance = this->state_covariance.block<3, 3>(6, 6);
            this->global_pose.global_rotation_covariance = this->state_covariance.block<3, 3>(9, 9);
            this->global_pose.global_angular_velocity_covariance = this->state_covariance.block<3, 3>(12, 12);
        }

        void LinearKalmanFilter::register_debug_publishers(ros::NodeHandle nh)
        {
            
        }

        void LinearKalmanFilter::initialize_internal_state()
        {
            while (this->imu_input_manager->num_unread_messages() < 20)
            {
                usleep(1000);
            }

            this->imu_input_manager->recompute(this->global_pose);
            this->global_pose.global_rotation = this->imu_input_manager->get_global_rotation();

            Eigen::Matrix<real_t, 3, 3> global_heading_covariance = this->imu_input_manager->get_global_heading_covariance();

            Eigen::Matrix<real_t, 3, 3> local_to_global_mat = this->roll_pitch_yaw_to_rotation_matrix(this->global_pose.global_rotation);
            Eigen::Matrix<real_t, 3, 3> local_to_global_mat_t = local_to_global_mat.transpose();

            // TODO: tune the initialization of the covariances.
            real_t position_sigma = 0.01;
            real_t velocity_sigma = 0.01;
            real_t acceleration_sigma = 0.01;
            real_t angular_velocity_sigma = 0.01;

            this->global_pose.global_position_covariance = local_to_global_mat * (Eigen::Matrix<real_t, 3, 3>::Identity() * position_sigma) * local_to_global_mat_t;
            this->global_pose.global_velocity_covariance = local_to_global_mat * (Eigen::Matrix<real_t, 3, 3>::Identity() * velocity_sigma) * local_to_global_mat_t;
            this->global_pose.global_acceleration_covariance = local_to_global_mat * (Eigen::Matrix<real_t, 3, 3>::Identity() * acceleration_sigma) * local_to_global_mat_t;
            this->global_pose.global_rotation_covariance = this->imu_input_manager->get_global_heading_covariance();
            this->global_pose.global_angular_velocity_covariance = local_to_global_mat * (Eigen::Matrix<real_t, 3, 3>::Identity() * angular_velocity_sigma) * local_to_global_mat_t;

            this->state = Eigen::Matrix<real_t, state_dimension, 1>::Zero();

            this->state(9) = this->global_pose.global_rotation[0];
            this->state(10) = this->global_pose.global_rotation[1];
            this->state(11) = this->global_pose.global_rotation[2];

            this->state_covariance.setZero();

            this->state_covariance.block<3,3>(0, 0) = this->global_pose.global_position_covariance;
            this->state_covariance.block<3,3>(3, 3) = this->global_pose.global_velocity_covariance;
            this->state_covariance.block<3,3>(6, 6) = this->global_pose.global_acceleration_covariance;
            this->state_covariance.block<3,3>(9, 9) = this->global_pose.global_rotation_covariance;
            this->state_covariance.block<3,3>(12, 12) = this->global_pose.global_angular_velocity_covariance;
        }

        Eigen::Matrix<real_t, 3, 1> LinearKalmanFilter::rotate_vector(
                const Eigen::Matrix<real_t, 3, 3> &rotation_matrix,
                const Eigen::Matrix<real_t, 3, 3> &original_vector)
        {
            return rotation_matrix * original_vector;
        }

        Eigen::Matrix<real_t, 3, 3> LinearKalmanFilter::rotate_covariance_matrix(
                const Eigen::Matrix<real_t, 3, 3> &rotation_matrix, 
                const Eigen::Matrix<real_t, 3, 3> &original_covariance)
        {
            return rotation_matrix * original_covariance * (rotation_matrix.transpose());
        }

        Eigen::Matrix<real_t, 3, 3> LinearKalmanFilter::roll_pitch_yaw_to_rotation_matrix(const vector3r_t &rpy_vec)
        {
            // TODO: this isn't the most efficient, as it creates AA, then creates a quat, then goes to rot matrix. 
            // Not sure if it will ultimately matter, though.
            Eigen::AngleAxis<real_t> roll_angle(rpy_vec[0], Eigen::Matrix<real_t, 3, 1>::UnitZ());
            Eigen::AngleAxis<real_t> pitch_angle(rpy_vec[1], Eigen::Matrix<real_t, 3, 1>::UnitY());
            Eigen::AngleAxis<real_t> yaw_angle(rpy_vec[2], Eigen::Matrix<real_t, 3, 1>::UnitX());

            return (roll_angle * pitch_angle * yaw_angle).toRotationMatrix();
        }

        void LinearKalmanFilter::initialize_parameters_from_file(std::string parameter_path)
        {
            std::ifstream input_stream(parameter_path);

            this->imu_input_manager = std::make_unique<ImuInputManager>(this->read_matrix_from_file(input_stream, "imu_covariance"));
            this->gps_input_manager = std::make_unique<GpsInputManager>(this->read_matrix_from_file(input_stream, "gps_covariance"));

            std::vector<DriveInputManagerControlPoint> drive_control_points;
            drive_control_points.push_back(this->read_drive_control_point_from_file(input_stream, "forward_control_point"));
            drive_control_points.push_back(this->read_drive_control_point_from_file(input_stream, "turn_control_point"));

            this->drive_input_manager = std::make_unique<DriveInputManager>(drive_control_points);
        }

        DriveInputManagerControlPoint LinearKalmanFilter::read_drive_control_point_from_file(std::ifstream &stream, std::string expected_point_name)
        {
            std::string line;
            std::getline(stream, line);

            if (line != expected_point_name)
            {
                throw std::runtime_error("Unexpected matrix name: " + line + ". Expected " + expected_point_name);
            }

            real_t left_control;
            real_t right_control;
            std::getline(stream, line);
            std::istringstream control_stream(line);
            if (!(control_stream >> left_control >> right_control))
            {
                throw std::runtime_error("Unexpected line. Expected two floats for motor control, got " + line + ".");
            }

            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> control_matrix = this->read_matrix_from_file(stream, "");
            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> covariance_matrix = this->read_matrix_from_file(stream, "");

            DriveInputManagerControlPoint output(
                    std::make_pair(left_control, right_control),
                    control_matrix, 
                    covariance_matrix);

            return output;
        }

        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> LinearKalmanFilter::read_matrix_from_file(std::ifstream &stream, std::string expected_matrix_name)
        {
            std::string line;

            if (expected_matrix_name.size() > 0)
            {
                std::getline(stream, line);

                if (line != expected_matrix_name)
                {
                    throw std::runtime_error("Unexpected matrix name: " + line + ". Expected " + expected_matrix_name);
                }
            }

            int width;
            int height;
            std::getline(stream, line);
            std::istringstream matrix_size_line(line);

            if (!(matrix_size_line >> height >> width))
            {
                throw std::runtime_error("Expected height, width for matrix " + expected_matrix_name + ", but got " + line);
            }

            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> output_matrix;
            output_matrix.resize(height, width);

            for (int y = 0; y < height; y++)
            {
                std::getline(stream, line);
                std::istringstream matrix_data_line(line);
                real_t next_value = -1;

                for (int x = 0; x < width; x++)
                {
                    if (!(matrix_data_line >> next_value))
                    {
                        throw std::runtime_error("Malformed row. Not enough numbers in row: " + line);
                    }

                    output_matrix(y,x) = next_value;
                }

                if (matrix_data_line.rdbuf()->in_avail() > 0)
                {
                    throw std::runtime_error("Malformed row. Too many numbers in row.: " + line);
                }

            }

            return output_matrix;
        }
    }
}
