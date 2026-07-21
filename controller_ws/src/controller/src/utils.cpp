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

    std::vector<double> computeCornerVelocity(const std::vector<double>& k, const VehicleParams& config_){
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

    std::vector<double> computeDeltaS(const std::vector<Eigen::Vector2d>& spline_points_){
        size_t n = spline_points_.size();
        
        std::vector<double> s(n); 

        for(size_t i = 0; i < n; i++){
            const std::size_t next = (i + 1) % n;
            s[i] = (spline_points_[next] - spline_points_[i]).norm();
        }
        return s;
    }

    std::vector<double> getCumulativeS(const std::vector<Eigen::Vector2d>& spline_points_){
        std::vector<double> s_d = computeDeltaS(spline_points_);

        std::vector<double> s(spline_points_.size(), 0.0); // 1st point in latest_track_center should start from 0. 
        for(size_t i = 1; i<spline_points_.size(); i++){
            s[i] = s[i - 1] + s_d[i - 1];
        }

        return s;
    } 

    std::vector<double> computeSmoothVel(const std::vector<Eigen::Vector2d>& spline_points_, const std::vector<double>& spline_curvature_, 
                                        const VehicleParams& config_, double current_speed, 
                                        std::vector<double>& v_corner, std::vector<double>& v_accln, std::vector<double>& v_brake){
        // size of the spline samples
        size_t n = spline_points_.size();
        
        std::cout<<"Entered Compute smooth velocity"<<'\n'; 
        
        // fetch k
        std::vector<double> k = spline_curvature_;

        // get vcornering
        v_corner = computeCornerVelocity(k, config_);  // size n , 98

        // compute delta s
        auto s = computeDeltaS(spline_points_); // size n 

        // Acceleration profile
        v_accln = v_corner; 

        for (int iter = 0; iter < 3; iter++) {
            for (size_t i = 1; i < n; i++) { // i = 1
                double v_i = v_accln[i-1]; // 0 
                double ay_used = std::min(std::abs(k[i-1]) * v_i * v_i, config_.ay_max); // k[0] * v[0] * v[0], move from i-1 to i
                double pct = ay_used / config_.ay_max; 
                double ax_avail = config_.ax_max_accln * std::sqrt(std::max(0.0, 1.0 - pct*pct));
                
                double v_next = std::sqrt(2 * ax_avail * s[i-1] + v_i * v_i); // s[0] = length of segment from 0 to 1.  
                v_accln[i] = std::min(v_next, v_corner[i]); 
            }

            // closed loop continuation 
            double v_last = v_accln[n-1];
            double ay_used_wrap = std::min(std::abs(k[n-1]) * v_last * v_last, config_.ay_max);
            double pct_wrap = ay_used_wrap / config_.ay_max;
            double ax_avail_wrap = config_.ax_max_accln * std::sqrt(std::max(0.0, 1.0 - pct_wrap*pct_wrap));
            
            // Cap the wrap-around too
            double v_wrap_next = std::sqrt(2 * ax_avail_wrap * s[n-1] + v_last * v_last);
            v_accln[0] = std::min(v_wrap_next, v_corner[0]);
        }

        // Braking profile
        v_brake = v_corner; 

        for (int iter = 0; iter < 3; iter++) {
            for (int j = (int)n - 2; j >= 0; j--) {
                double v_j = v_brake[j+1];
                double ay_used = std::min(std::abs(k[j+1]) * v_j * v_j, config_.ay_max);
                double pct = ay_used / config_.ay_max;
                double ax_avail = std::max(0.0, config_.ax_max_brake * std::sqrt(std::max(0.0, 1.0 - pct*pct)));
                
                // Calculate the new speed, but cap it at the cornering limit
                double v_prev = std::sqrt(2 * ax_avail * s[j] + v_j * v_j);
                v_brake[j] = std::min(v_prev, v_corner[j]);
            }

            // Braking Profile Wrap Continuation
            double v_first = v_brake[0];
            double ay_used_wrap = std::min(std::abs(k[0]) * v_first * v_first, config_.ay_max); 
            double pct_wrap = ay_used_wrap / config_.ay_max;
            double ax_avail_wrap = std::max(0.0, config_.ax_max_brake * std::sqrt(std::max(0.0, 1.0 - pct_wrap*pct_wrap)));

            // Cap the wrap-around too
            double v_wrap_prev = std::sqrt(2 * ax_avail_wrap * s[n-1] + v_first * v_first);
            v_brake[n-1] = std::min(v_wrap_prev, v_corner[n-1]);
        }
        
        // Final profile
        std::vector<double> v_profile(k.size()); 
        v_profile[0] = std::min({v_corner[0], v_accln[0], v_brake[0]});

        for(size_t x = 1; x<n; x++){
            v_profile[x] = std::min({v_corner[x], v_accln[x], v_brake[x]});
        }

        std::cout<<"Finished Compute smooth velocity"<<'\n'; 
        
        return v_profile; 
    }

    double computeFeedforward(double target_acceleration, double target_velocity, const VehicleParams& config_){
        
        const double Fdrag = 0.5 * config_.rho_air * config_.CdA* target_velocity * target_velocity;
        const double Froll = config_.Crr * config_.mass * config_.g; 

        const double max_drive_force = config_.max_drive_torque / config_.wheel_radius; 
        const double max_brake_force = config_.max_brake_torque / config_.wheel_radius; 

        const double actuator_force = config_.mass * target_acceleration + Fdrag + Froll;

        double feedforward = 0.0;

        if(actuator_force>=0.0){
            feedforward = actuator_force / max_drive_force; 
        }
        else{
            feedforward = actuator_force / max_brake_force; 
        }
        
        return feedforward; 
    }

    // FAHHHHHHHHHHHH
    std::vector<double> computeSmoothVelFromCSV(
        const std::string& input_filename,
        const std::string& output_filename,
        const VehicleParams& config_,
        std::vector<double>& cumulative_s,
        std::vector<double>& spline_curvature,
        std::vector<double>& v_corner,
        std::vector<double>& v_accln,
        std::vector<double>& v_brake)
    {
        cumulative_s.clear();
        spline_curvature.clear();
        v_corner.clear();
        v_accln.clear();
        v_brake.clear();

        /*
        * Read:
        *
        * Cumulative_S,Spline_Curvature,Target_Velocity
        */
        std::ifstream input_file(input_filename);

        if (!input_file.is_open()) {
            std::cerr
                << "Could not open spline-curvature CSV: "
                << input_filename
                << '\n';

            return {};
        }

        std::string line;

        // Skip header.
        std::getline(input_file, line);

        while (std::getline(input_file, line)) {
            if (line.empty()) {
                continue;
            }

            std::stringstream line_stream(line);

            std::string cumulative_s_text;
            std::string curvature_text;
            std::string old_target_velocity_text;

            std::getline(
                line_stream,
                cumulative_s_text,
                ','
            );

            std::getline(
                line_stream,
                curvature_text,
                ','
            );

            // The existing Target_Velocity column is ignored.
            std::getline(
                line_stream,
                old_target_velocity_text,
                ','
            );

            try {
                double s_value =
                    std::stod(cumulative_s_text);

                double curvature_value =
                    std::stod(curvature_text);
                
                constexpr double s_tolerance = 1e-9;

                if (!cumulative_s.empty()) {
                    const double previous_s = cumulative_s.back();

                    if (s_value < previous_s - s_tolerance) {
                        std::cerr
                            << "Cumulative_S reset detected: "
                            << previous_s
                            << " -> "
                            << s_value
                            << ". Keeping only the first lap.\n";

                        break;
                    }

                    if (std::abs(s_value - previous_s) <= s_tolerance) {
                        std::cerr
                            << "Skipping duplicate Cumulative_S: "
                            << s_value
                            << '\n';

                        continue;
                    }
                }

                cumulative_s.push_back(s_value);
                spline_curvature.push_back(curvature_value);
            }
            catch (const std::exception& exception) {
                std::cerr
                    << "Skipping invalid CSV row: "
                    << line
                    << '\n';
            }
        }

        input_file.close();

        const size_t n = spline_curvature.size();

        if (n < 2 || cumulative_s.size() != n) {
            std::cerr
                << "Invalid spline-curvature CSV data."
                << '\n';

            return {};
        }

        /*
        * Compute delta_s from Cumulative_S.
        *
        * For every ordinary segment:
        *
        * delta_s[i] = S[i+1] - S[i]
        */
        std::vector<double> delta_s(n, 0.0);

        for (size_t i = 0; i < n - 1; i++) {
            delta_s[i] =
                cumulative_s[i + 1] -
                cumulative_s[i];

                if (delta_s[i] <= 0.0) {
                    std::cerr
                        << "Cumulative_S is not strictly increasing at index "
                        << i
                        << ": "
                        << cumulative_s[i]
                        << " -> "
                        << cumulative_s[i + 1]
                        << '\n';

                    return {};
                }
        }

        /*
        * The CSV appears to contain the full closed spline, so estimate
        * the final wrap segment using the average nearby spacing.
        *
        * This segment is only used for the closed-loop continuation:
        * final point -> first point.
        */
        const size_t spacing_sample_count =
            std::min<size_t>(100, n - 1);

        double spacing_sum = 0.0;

        for (size_t i = n - 1 - spacing_sample_count;
            i < n - 1;
            i++)
        {
            spacing_sum +=
                cumulative_s[i + 1] -
                cumulative_s[i];
        }

        delta_s[n - 1] =
            spacing_sum /
            static_cast<double>(spacing_sample_count);

        /*
        * Use the existing computeCornerVelocity() function.
        *
        * No duplicated cornering-speed formula is needed here.
        */
        v_corner =
            computeCornerVelocity(
                spline_curvature,
                config_
            );

        /*
        * Forward acceleration pass.
        *
        * This is the same logic as your existing computeSmoothVel().
        */
        v_accln = v_corner;

        for (int iter = 0; iter < 3; iter++) {
            for (size_t i = 1; i < n; i++) {
                double v_i =
                    v_accln[i - 1];

                double ay_used =
                    std::min(
                        std::abs(spline_curvature[i - 1]) *
                        v_i *
                        v_i,
                        config_.ay_max
                    );

                double pct =
                    ay_used /
                    config_.ay_max;

                double ax_avail =
                    config_.ax_max_accln *
                    std::sqrt(
                        std::max(
                            0.0,
                            1.0 - pct * pct
                        )
                    );

                double v_next =
                    std::sqrt(
                        v_i * v_i +
                        2.0 *
                        ax_avail *
                        delta_s[i - 1]
                    );

                v_accln[i] =
                    std::min(
                        v_next,
                        v_corner[i]
                    );
            }

            /*
            * Closed-loop acceleration continuation.
            */
            double v_last =
                v_accln[n - 1];

            double ay_used_wrap =
                std::min(
                    std::abs(spline_curvature[n - 1]) *
                    v_last *
                    v_last,
                    config_.ay_max
                );

            double pct_wrap =
                ay_used_wrap /
                config_.ay_max;

            double ax_avail_wrap =
                config_.ax_max_accln *
                std::sqrt(
                    std::max(
                        0.0,
                        1.0 -
                        pct_wrap *
                        pct_wrap
                    )
                );

            double v_wrap_next =
                std::sqrt(
                    v_last *
                    v_last +
                    2.0 *
                    ax_avail_wrap *
                    delta_s[n - 1]
                );

            v_accln[0] =
                std::min(
                    v_wrap_next,
                    v_corner[0]
                );
        }

        /*
        * Backward braking pass.
        *
        * This is the same logic as your existing computeSmoothVel().
        */
        v_brake = v_corner;

        for (int iter = 0; iter < 3; iter++) {
            for (int j = static_cast<int>(n) - 2;
                j >= 0;
                j--)
            {
                double v_j =
                    v_brake[j + 1];

                double ay_used =
                    std::min(
                        std::abs(spline_curvature[j + 1]) *
                        v_j *
                        v_j,
                        config_.ay_max
                    );

                double pct =
                    ay_used /
                    config_.ay_max;

                double ax_avail =
                    std::max(
                        0.0,
                        config_.ax_max_brake *
                        std::sqrt(
                            std::max(
                                0.0,
                                1.0 - pct * pct
                            )
                        )
                    );

                double v_previous =
                    std::sqrt(
                        v_j * v_j +
                        2.0 *
                        ax_avail *
                        delta_s[j]
                    );

                v_brake[j] =
                    std::min(
                        v_previous,
                        v_corner[j]
                    );
            }

            /*
            * Closed-loop braking continuation.
            */
            double v_first =
                v_brake[0];

            double ay_used_wrap =
                std::min(
                    std::abs(spline_curvature[0]) *
                    v_first *
                    v_first,
                    config_.ay_max
                );

            double pct_wrap =
                ay_used_wrap /
                config_.ay_max;

            double ax_avail_wrap =
                std::max(
                    0.0,
                    config_.ax_max_brake *
                    std::sqrt(
                        std::max(
                            0.0,
                            1.0 -
                            pct_wrap *
                            pct_wrap
                        )
                    )
                );

            double v_wrap_previous =
                std::sqrt(
                    v_first *
                    v_first +
                    2.0 *
                    ax_avail_wrap *
                    delta_s[n - 1]
                );

            v_brake[n - 1] =
                std::min(
                    v_wrap_previous,
                    v_corner[n - 1]
                );
        }

        /*
        * Final profile:
        *
        * minimum of cornering, acceleration and braking profiles.
        */
        std::vector<double> v_profile(n);

        for (size_t i = 0; i < n; i++) {
            v_profile[i] =
                std::min({
                    v_corner[i],
                    v_accln[i],
                    v_brake[i]
                });
        }

        /*
        * Save everything for plotting.
        */
        std::ofstream output_file(output_filename);

        if (!output_file.is_open()) {
            std::cerr
                << "Could not create output CSV: "
                << output_filename
                << '\n';

            return v_profile;
        }

        output_file
            << "Index,"
            << "Cumulative_S,"
            << "Delta_S_To_Next,"
            << "Spline_Curvature,"
            << "Velocity_Corner,"
            << "Velocity_Accel,"
            << "Velocity_Brake,"
            << "Target_Velocity\n";

        output_file << std::setprecision(12);

        for (size_t i = 0; i < n; i++) {
            output_file
                << i << ","
                << cumulative_s[i] << ","
                << delta_s[i] << ","
                << spline_curvature[i] << ","
                << v_corner[i] << ","
                << v_accln[i] << ","
                << v_brake[i] << ","
                << v_profile[i]
                << '\n';
        }

        output_file.close();

        std::cout
            << "Dense velocity profile generated using "
            << n
            << " spline samples."
            << '\n';

        std::cout
            << "Saved profile to: "
            << output_filename
            << '\n';

        return v_profile;
    }
    
    // Used AI for plotting files 
    void saveProfileAnalysisToCSV(
        const std::vector<Eigen::Vector2d>& spline_points,
        const std::vector<double>& spline_curvature,
        const VehicleParams& config_,
        const std::vector<double>& v_profile,
        const std::vector<double>& v_accln,
        const std::vector<double>& v_brake,
        const std::vector<double>& v_corner,
        const std::string& filename)
    {
        const std::size_t n =
            spline_points.size();

        if (n < 2 ||
            spline_curvature.size() != n ||
            v_profile.size() != n ||
            v_accln.size() != n ||
            v_brake.size() != n ||
            v_corner.size() != n)
        {
            std::cerr
                << "Invalid dense-profile vector sizes!\n";

            return;
        }

        const std::vector<double> delta_s =
            computeDeltaS(spline_points);

        const std::vector<double> cumulative_s =
            getCumulativeS(spline_points);

        const std::vector<double>& k =
            spline_curvature;

        const double track_length =
            cumulative_s.back() + delta_s.back();

        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr
                << "Failed to open profile analysis CSV!"
                << std::endl;
            return;
        }

        file <<
            "Index,"
            "Cumulative_S,"
            "Delta_S_To_Next,"
            "Velocity_Profile,"
            "Velocity_Accel,"
            "Velocity_Brake,"
            "Velocity_Corner,"
            "Curvature,"
            "Segment_Acceleration,"
            "Accel_Limit,"
            "Brake_Limit,"
            "Segment_Feasible,"
            "Corner_Feasible\n";

        constexpr double tolerance = 1e-5;

        for (size_t i = 0; i < n; i++) {

            size_t next = (i + 1) % n;

            double ds = delta_s[i];

            double v_i = v_profile[i];
            double v_next = v_profile[next];

            /*
            * Signed average acceleration required over segment i -> next:
            *
            * v_next² = v_i² + 2*a*delta_s
            */
            double segment_acceleration = 0.0;

            if (ds > 1e-8) {
                segment_acceleration =
                    (v_next * v_next - v_i * v_i)
                    / (2.0 * ds);
            }

            /*
            * Acceleration limit at the beginning of the segment.
            * This matches the forward-pass calculation.
            */
            double ay_accel = std::min(
                std::abs(k[i]) * v_i * v_i,
                config_.ay_max
            );

            double accel_pct =
                ay_accel / config_.ay_max;

            double accel_limit =
                config_.ax_max_accln *
                std::sqrt(
                    std::max(
                        0.0,
                        1.0 - accel_pct * accel_pct
                    )
                );

            /*
            * Braking limit at the end of the segment.
            * This matches the backward-pass calculation.
            */
            double ay_brake = std::min(
                std::abs(k[next]) * v_next * v_next,
                config_.ay_max
            );

            double brake_pct =
                ay_brake / config_.ay_max;

            double brake_limit =
                config_.ax_max_brake *
                std::sqrt(
                    std::max(
                        0.0,
                        1.0 - brake_pct * brake_pct
                    )
                );

            bool segment_feasible;

            if (segment_acceleration >= 0.0) {
                segment_feasible =
                    segment_acceleration
                    <= accel_limit + tolerance;
            }
            else {
                segment_feasible =
                    std::abs(segment_acceleration)
                    <= brake_limit + tolerance;
            }

            bool corner_feasible =
                v_i <= v_corner[i] + tolerance &&
                v_next <= v_corner[next] + tolerance;

            file
                << i << ","
                << cumulative_s[i] << ","
                << ds << ","
                << v_profile[i] << ","
                << v_accln[i] << ","
                << v_brake[i] << ","
                << v_corner[i] << ","
                << k[i] << ","
                << segment_acceleration << ","
                << accel_limit << ","
                << brake_limit << ","
                << segment_feasible << ","
                << corner_feasible << "\n";
        }

        /*
        * Append point zero at the track length so the velocity plots
        * visibly close the loop.
        *
        * Segment-related columns are left empty because this row is
        * only used for plotting the closing point.
        */
        file
            << n << ","
            << track_length << ",,"
            << v_profile[0] << ","
            << v_accln[0] << ","
            << v_brake[0] << ","
            << v_corner[0] << ","
            << k[0]
            << ",,,,,\n";

        file.close();

        std::cout
            << "Profile analysis saved to: "
            << filename
            << '\n';
    }

}