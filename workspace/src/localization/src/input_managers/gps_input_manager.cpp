#include "input_managers/gps_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        GpsInputManager::GpsInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&measurement_covariance)
        {
           this->unread_message_count = 0; 
           pthread_mutex_init(&this->processing_mutex, NULL);

           this->latitude_tableau = 0;
           this->longitude_tableau = 0;
           this->altitude_tableau = 0;

           this->global_position_covariance = std::move(measurement_covariance);

           // TODO: Assuming that position covariance is static. 
           // This probably needs to be revisited.
           this->last_global_position_covariance = this->global_position_covariance;
        }

        GpsInputManager::~GpsInputManager()
        {
            pthread_mutex_destroy(&this->processing_mutex);
        }

        const vector3r_t& GpsInputManager::get_global_position() const
        {
            return this->last_global_position;
        }

        const Eigen::Matrix<real_t, 3, 3>GpsInputManager::get_global_position_covariance() const
        {
            return this->last_global_position_covariance;
        }

        int GpsInputManager::num_unread_messages()
        {
            return this->unread_message_count;
        }

        void GpsInputManager::add_gps_message(const sensor_msgs::NavSatFix::ConstPtr &msg)
        {
            pthread_mutex_lock(&this->processing_mutex);
            real_t top_mult = static_cast<real_t>(this->unread_message_count);
            this->unread_message_count++;
            real_t bottom_div = static_cast<real_t>(this->unread_message_count);
            
            this->latitude_tableau = (((this->latitude_tableau * top_mult) + msg->latitude) / bottom_div);
            this->longitude_tableau = (((this->longitude_tableau * top_mult) + msg->longitude) / bottom_div);
            this->altitude_tableau = (((this->altitude_tableau * top_mult) + msg->altitude) / bottom_div);
            pthread_mutex_unlock(&this->processing_mutex);
        }

        void GpsInputManager::recompute(const GlobalPose &global_pose)
        {
            pthread_mutex_lock(&this->processing_mutex);
            
            geographic_msgs::GeoPoint geo_pt;
            geo_pt.latitude = this->latitude_tableau;
            geo_pt.longitude = this->longitude_tableau;
            geo_pt.altitude = this->altitude_tableau;

            // TODO: this is a bit of a hack.
            // We could theoretically run into problems if we cross UTM bands
            // See explanation here: https://answers.ros.org/question/50763/need-help-converting-lat-long-coordinates-into-meters/
            //
            // A potential workaround could be to use this library: https://proj.org/about.html
            geodesy::UTMPoint utm_pt(geo_pt);
            last_global_position(0, 0) = utm_pt.northing;
            last_global_position(1, 0) = utm_pt.easting;
            last_global_position(2, 0) = utm_pt.altitude;

            // TODO: This code assumes that position covariance is static. 
            // It probably needs to be revisited.

            this->unread_message_count = 0;
            this->latitude_tableau = 0;
            this->longitude_tableau = 0;
            this->altitude_tableau = 0;
            pthread_mutex_unlock(&this->processing_mutex);
        }
    }
}
