#include "utils.h"

namespace utils{
    
    std::vector<double> computeCurvature(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_){
        size_t n = latest_track_center.size();
        std::vector<double> k(n);  
        
        for(size_t i = 0; i < n; i++){
            auto& p1 = latest_track_center[(i - 1 + n) % n]; // this type of indexing helps us with considering the track as closed loop. 
            auto& p2 = latest_track_center[i];
            auto& p3 = latest_track_center[(i + 1) % n];
            
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
        // k[0] = k[1];
        // k[n-1] = k[n-2];

        // Smooth k 
        std::vector<double> k_smooth = k;
        for (size_t i = 0; i < n; i++) {
            k_smooth[i] = (k[(i-2+n)%n] + k[(i-1+n)%n] + k[i] + k[(i+1)%n] + k[(i+2)%n]) / 5.0; // closed loop wrap again 
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
        size_t n = latest_track_center.size();
        
        std::vector<double> s(n); 

        for(size_t i = 0; i < n; i++){
            const auto& p1 = latest_track_center[i];
            const auto& p2 = latest_track_center[(i + 1) % n]; // Automatically wraps to 0 at the end
            s[i] = std::hypot(p2.x - p1.x, p2.y - p1.y); 
        }
        return s;
    }

    std::vector<double> computeSmoothVel(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_,
                                        double current_speed, std::vector<double>& v_corner, std::vector<double>& v_accln, std::vector<double>& v_brake){
        size_t n = latest_track_center.size();
        
        // get k 
        auto k = computeCurvature(latest_track_center, config_);  // size n, 98

        // get vcornering
        v_corner = computeVelocity(k, config_);  // size n , 98

        // compute delta s
        auto s = computeDeltaS(latest_track_center); // size n 

        // Acceleration profile
        v_accln.assign(n, 0.0);
        //v_accln[0] = v_corner[0];
        v_accln[0] = v_corner[0]; 

        for (int iter = 0; iter < 3; iter++) {
            for (size_t i = 1; i < n; i++) {
                double v_i = v_accln[i-1];
                double ay_used = std::min(std::abs(k[i-1]) * v_i * v_i, config_.ay_max);
                double pct = ay_used / config_.ay_max;
                double ax_avail = config_.ax_max_accln * std::sqrt(std::max(0.0, 1.0 - pct*pct));
                v_accln[i] = std::min(config_.v_max, std::sqrt(2*ax_avail*s[i-1] + v_i*v_i));
            }

            // closed loop continuation 
            double v_last = v_accln[n-1];
            double ay_used_wrap = std::min(std::abs(k[n-1]) * v_last * v_last, config_.ay_max);
            double pct_wrap = ay_used_wrap / config_.ay_max;
            double ax_avail_wrap = config_.ax_max_accln * std::sqrt(std::max(0.0, 1.0 - pct_wrap*pct_wrap));
            // Use s[n-1] because it naturally represents the segment connecting index n-1 to index 0!
            v_accln[0] = std::min(config_.v_max, std::sqrt(2*ax_avail_wrap*s[n-1] + v_last*v_last));
        }

        // Braking profile
        v_brake.assign(n, 0.0);
        v_brake[n-1] = v_corner[n-1];

        for (int iter = 0; iter < 3; iter++) {
            for (int j = (int)n - 2; j >= 0; j--) {
                double v_j = v_brake[j+1];
                double ay_used = std::min(std::abs(k[j+1]) * v_j * v_j, config_.ay_max);
                double pct = ay_used / config_.ay_max;
                double ax_avail = std::max(0.0, config_.ax_max_brake * std::sqrt(std::max(0.0, 1.0 - pct*pct)));
                v_brake[j] = std::min(config_.v_max, std::sqrt(2*ax_avail*s[j] + v_j*v_j));
            }

            // Braking Profile Wrap Continuation
            double v_first = v_brake[0];
            // Use the velocity at n-1 alongside the curvature at n-1
            double ay_used_wrap = std::min(std::abs(k[n-1]) * v_brake[n-1] * v_brake[n-1], config_.ay_max);
            double pct_wrap = ay_used_wrap / config_.ay_max;
            double ax_avail_wrap = std::max(0.0, config_.ax_max_brake * std::sqrt(std::max(0.0, 1.0 - pct_wrap*pct_wrap)));

            // Retard from v_first back into v_brake[n-1]
            v_brake[n-1] = std::min(config_.v_max, std::sqrt(2 * ax_avail_wrap * s[n-1] + v_first * v_first));
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