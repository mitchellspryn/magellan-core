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

        void LinearKalmanFilter::accept_gps_message(sensor_msgs::NavSatfix &msg)
        {
            this->gps_input_manager->add_gps_message(msg);
        }

        void LinearKalmanFilter::accept_drive_message(magellan_messages::MsgMagellanDrive &msg)
        {
            this->drive_input_manager->add_drive_message(msg);
        }

        void LinearKalmanFilter::update_global_pose(ros::Duration duration)
        {
            // TODO:
            // Get sensor readings / covariances (in body frames)
            //
            // Rotate sensor readings to global frame
            //
            // Rotate uncertainty matrix to global frame (via toRotationMatrix())
            //
            // Run iteration of kalman
        }

        void LinearKalmanFilter::register_debug_publishers(ros::NodeHandle nh)
        {
            
        }

        void LinearKalmanFilter::initialize_internal_state()
        {
            while (this->imu_input_manager->num_received_messages() < 20)
            {
                usleep(1000);
            }

            this->imu_input_manager->recompute();
            this->global_pose->global_rotation = this->imu_input_manager->get_global_heading();

            // TODO: tune the initialization of the covariances.
            real_t position_sigma = 0.01;
            real_t rotation_sigma = 0.01;

            for (int i = 0; i < 3; i++)
            {
                this->global_pose->global_position_covariance(i,i) = position_sigma;
            }

            for (int i = 0; i < 4; i++)
            {
                this->global_pose->global_rotation_covariance(i,i) = rotation_sigma;
            }

            for (int i = 0; i < state_dimension; i++)
            {
                this->state(i) = 0;
            }

            this->state(9) = this->global_pose->global_rotation.w();
            this->state(10) = this->global_pose->global_rotation.x();
            this->state(11) = this->global_pose->global_rotation.y();
            this->state(12) = this->global_pose->global_rotation.z();

            this->state_covariance.setZero();

            for (int i = 0; i < state_dimension; i++)
            {
                if (i < 9)
                {
                    this->state_covariance(i,i) = position_sigma;
                }
                else
                {
                    this->state_covariance(i,i) = rotation_sigma;
                }
            }
        }

        void LinearKalmanFilter::initialize_parameters_from_file(std::string parameter_path)
        {
            std::ifstream input_stream(parameter_path);

            this->imu_input_manager = std::make_unique<ImuInputManager>(this->read_matrix_from_file(input_stream, "imu_covariance"));
            this->gps_input_manager = std::make_unique<GpsInputManager>(this->read_matrix_from_file(input_stream, "gps_covariance"));

            std::vector<DriveInputControlPoint> drive_control_points;
            drive_control_points.push_back(this->read_drive_control_point_from_file(input_stream, "forward_control_point"));
            drive_control_points.push_back(this->read_drive_control_point_from_file(input_stream, "turn_control_point"));

            this->drive_input_manager = std::make_unique<DriveInputManager>(drive_control_points);
        }

        DriveInputControlPoint LinearKalmanFilter::read_drive_control_point_from_file(std::ifstream &stream, std::string &expected_matrix_name)
        {
            std::string line;
            std::getline(stream, line);

            if (line != expected_matrix_name)
            {
                throw std::runtime_error("Unexpected matrix name: " + line + ". Expected " + expected_matrix_name);
            }

            real_t left_control;
            real_t right_control;
            std::getline(stream, line);
            istringstream control_stream(line);
            if (!(line >> left_control >> right_control))
            {
                throw std::runtime_error("Unexpected line. Expected two floats for motor control, got " + line + ".");
            }

            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> control_matrix = this->read_matrix_from_file(stream, "");
            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> covariance_matrix = this->read_matrix_from_file(stream, "");

            DriveInputControlPoint output(
                    std::make_pair(left_control, right_control),
                    control_matrix, 
                    covariance_matrix);

            return output;
        }

        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> LinearKalmanFilter::read_matrix_from_file(std::ifstream &stream, std::string &expected_matrix_name)
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
            istringstream matrix_size_line(line);

            if (!(matrix_size_line >> height >> width))
            {
                throw std::runtime_error("Expected height, width for matrix " + expected_matrix_name + ", but got " + line);
            }

            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> output_matrix;
            output_matrix.resize(height, width);

            for (int y = 0; y < height; y++)
            {
                std::getline(stream, line);
                istringstream matrix_data_line(line);
                real_t next_value = -1;

                for (int x = 0; x < width; x++)
                {
                    if (!(matrix_data_line >> next_value))
                    {
                        throw std::runtime_error("Malformed row. Not enough numbers in row: " + line);
                    }

                    matrix(y,x) = next_value;
                }

                if (matrix_data_line.rdbuf()->in_avail() > 0)
                {
                    throw std::runtime_error("Malformed row. Too many numbers in row.: " + line);
                }
            }

            return matrix;
        }
    }
}
