#include "utils.h"

namespace utils{
    
    std::vector<double> computeCurvature(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_){
        size_t n = latest_track_center.size();
        std::vector<double> k(n);  
        
        for(size_t i = 1; i<n-1; i++){ // .size() returns size_t. 
            auto& p1 = latest_track_center[i-1];
            auto& p2 = latest_track_center[i];
            auto& p3 = latest_track_center[i+1];
            
            double a = std::hypot(p2.x - p3.x, p2.y - p3.y); 
            double b = std::hypot(p1.x - p3.x, p1.y - p3.y); 
            double c = std::hypot(p1.x - p2.x, p1.y - p2.y);

            double s = (a+b+c)/2; 

            double area = std::sqrt(std::max(0.0, s*(s-a)*(s-b)*(s-c)));

            if(a*b*c > 1e-8){
                k[i] = (4.0 * area) / (a*b*c); 
            }
            else{
                k[i] = 0.0;
            }
        }
        k[0] = k[1];
        k[n-1] = k[n-2];

        // Smooth k 
        std::vector<double> k_smooth = k;
        for (size_t i = 2; i < n - 2; i++) {
            k_smooth[i] = (k[i-2] + k[i-1] + k[i] + k[i+1] + k[i+2]) / 5.0;
        }
        k = k_smooth;

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

    std::vector<double> computeDeltaS(const std::vector<geometry_msgs::msg::Point>& latest_track_center){
        if (latest_track_center.size() < 2) return {};

        // The size of delta_s is (N - 1) because it represents intervals between points
        std::vector<double> s(latest_track_center.size() - 1); 

        for(size_t i = 0; i < latest_track_center.size() - 1; i++){
            const auto& p1 = latest_track_center[i];
            const auto& p2 = latest_track_center[i+1];

            // Compute Euclidean distance
            s[i] = std::hypot(p2.x - p1.x, p2.y - p1.y); 
        }
        return s;
    }

    std::vector<double> computeSmoothVel(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_,
                                        std::vector<double>& v_corner, std::vector<double>& v_accln, std::vector<double>& v_brake){
        size_t n = latest_track_center.size();
        
        // get k 
        auto k = computeCurvature(latest_track_center, config_);  // size n, 98

        // get vcornering
        v_corner = computeVelocity(k, config_);  // size n , 98

        // compute delta s
        auto s = computeDeltaS(latest_track_center); // size n-1 , 97 

        // Acceleration profile
        v_accln.assign(n, 0.0);
        //v_accln[0] = v_corner[0];
        v_accln[0] = 0.0;  

        for(size_t i=1; i<n; i++){ // size n 
            double v_i = v_accln[i-1]; 

            double ay_used = std::min(std::abs(k[i-1]) * v_i * v_i, config_.ay_max);
            double ay_used_percentage = ay_used/config_.ay_max;
            double ax_avail = config_.ax_max_accln * std::sqrt(std::max(0.0, 1.0 - (ay_used_percentage * ay_used_percentage)));

            v_accln[i] = std::sqrt(2*ax_avail*s[i-1] + v_i * v_i); // s[i-1] as it has n-1 size, 0 to 96 
        }
        
        // Braking profile
        v_brake.assign(n, 0.0);
        v_brake[n-1] = std::min(v_corner[n-1], v_accln[n-1]);

        for(int j = (int)n - 2; j>=0; j--){
            double v_j = v_brake[j+1];

            double ay_used = std::min(std::abs(k[j+1]) * v_j * v_j, config_.ay_max);
            double ay_used_percentage = ay_used/config_.ay_max;
            double ax_avail = config_.ax_max_brake * std::sqrt(std::max(0.0, 1.0 - (ay_used_percentage * ay_used_percentage)));
            ax_avail = std::max(0.0, ax_avail);

            v_brake[j] = std::sqrt(2*ax_avail*s[j] + v_j * v_j); // s[j], 96 to 0 
        }
        
        // Final profile
        std::vector<double> v_profile(k.size()); 
        v_profile[0] = std::min({v_corner[0], v_accln[0], v_brake[0]});

        for(size_t x = 1; x<n; x++){
            v_profile[x] = std::min({v_corner[x], v_accln[x], v_brake[x]});
        }

        return v_profile; 
    }
    
    // Used AI for plotting files 
    void saveProfileToCSV(const std::vector<double>& v_profile, 
                        const std::vector<double>& v_accln, 
                        const std::vector<double>& v_brake, 
                        const std::vector<double>& v_corner,
                        const std::string& filename) 
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing CSV!" << std::endl;
            return;
        }

        // Header row with all channels
        file << "Index,Velocity_Profile,Velocity_Accel,Velocity_Brake,Velocity_Corner\n"; 
        
        for (size_t i = 0; i < v_profile.size(); ++i) {
            file << i << "," 
                << v_profile[i] << "," 
                << v_accln[i] << "," 
                << v_brake[i] << "," 
                << v_corner[i] << "\n";
        }
        file.close();
    }

}