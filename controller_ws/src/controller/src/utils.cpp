#include "utils.h"

namespace utils{
    
    std::vector<double> computeCurvature(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_){
        std::vector<double> k(latest_track_center.size());  
        for(size_t i = 0; i<latest_track_center.size(); i++){ // .size() returns size_t. 
            // TODO
        }
        return k; 
    }

    std::vector<double> computeVelocity(const std::vector<double>& k, const VehicleParams& config_){
        std::vector<double> velocity_cornering; 

        for(size_t i = 0; i<k.size(); i++){
            if(std::abs(k[i]) < 1e-5){
                velocity_cornering.push_back(config_.v_max); 
            }
            else{
                double v_curve = std::sqrt(config_.ay_max / std::abs(k[i]));
                velocity_cornering.push_back(std::min(v_curve, config_.v_max));
            }
        }    
        return velocity_cornering; 
    }
}