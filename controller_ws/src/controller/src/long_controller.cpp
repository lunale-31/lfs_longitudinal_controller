#include "long_controller.h"

Controller::Controller() : rclcpp::Node("longitudinal_controller"){

    state_subscriber = this->create_subscription<lfs_msgs::msg::BikeState>("/current_state", 10, std::bind(&Controller::controllerCallback, this, std::placeholders::_1)); 
    throttle_publisher = this->create_publisher<std_msgs::msg::Float64>("/throttle_cmd", 10); 

    control_debug_pub = this->create_publisher<lfs_msgs::msg::ControlDebug>("/control_debug", 10);
    profile_debug_pub = this->create_publisher<lfs_msgs::msg::ProfileDebug>("/profile_debug", 10);

    track_client = this->create_client<track_srv::srv::ReturnTrack>("/return_track");  // Enter the service name correctly when you create a client.

    RCLCPP_INFO(this->get_logger(), "Controller is active! waiting for simulation ticks..."); 
    
    loadParams();
    pid_1 = PID(kp, ti, td, tr, n, beta, dt);

    this->trackServiceRequest(); // To collect the track points just one time 
} 

void Controller::loadParams(){
    // Default declaration for fail-safe 
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ti", 0.1);
    this->declare_parameter<double>("td", 0.01);
    this->declare_parameter<double>("tr", 0.01);
    this->declare_parameter<double>("n", 1.0);
    this->declare_parameter<double>("beta", 1.0);
    this->declare_parameter<double>("dt", 0.05);

    // Grab the values from yaml file
    this->get_parameter("kp", kp);
    this->get_parameter("ti", ti);
    this->get_parameter("td", td);
    this->get_parameter("tr", tr);
    this->get_parameter("n", n);
    this->get_parameter("beta", beta);
    this->get_parameter("dt", dt);
}

void Controller::trackServiceRequest(){

    // Wait for service to become available
    while(!track_client->wait_for_service(1s)){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }    
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // Create a request message
    auto request = std::make_shared<track_srv::srv::ReturnTrack::Request>();

    // Get the response using asychronous callback to prevent controller callback loop freeze. 
    // [this] remembers the current Controller instance, called as capture clause. 
    track_client->async_send_request(request,
        [this](rclcpp::Client<track_srv::srv::ReturnTrack>::SharedFuture future) {
            /* OBTAINING TRACK POINTS */
            auto response = future.get();
            
            // Handle the response data 
            this->latest_track_center = response->points; 
            RCLCPP_INFO(this->get_logger(), "Track received. Controller ready for live profiling.");
            
            /* SPLINE INTERPOLATION */
            spline_control_points.clear();
            spline_points_.clear();
            spline_curvature_.clear();

            // Fit Bspline to the obtained points
            spline_control_points.reserve(latest_track_center.size());

            // For each point in latest_track_center (:)
            for(const auto& point : latest_track_center){
                spline_control_points.push_back(Eigen::Vector2d(point.x, point.y));
            }
            spline_ = std::make_unique<LoopingUniformCubicBSpline<Eigen::Vector2d>>(spline_control_points);

            // Get the sampled spline points via functions inside spline class
            const double max_t = static_cast<double>(spline_->getMaxT()); 
            constexpr double spline_sample_t = 0.1;
            const std::size_t sample_count = static_cast<size_t>(std::ceil(max_t/spline_sample_t));
            
            spline_points_.reserve(sample_count); 
            spline_curvature_.reserve(sample_count);

            for(size_t i = 0; i < sample_count; i++){
                const double t = static_cast<double>(i) * spline_sample_t; 
                if(t>=max_t){
                    break;
                }
                const auto spline_data = spline_->getCurvature(t);  

                const Eigen::Vector2d& position_s = spline_data.position; 
                const Eigen::Vector2d& first_derivative = spline_data.tangent; 
                const Eigen::Vector2d& second_derivative = spline_data.curvature; 

                const double fd_squared = first_derivative.squaredNorm(); 

                // Curvature (K) formula 
                double curvature_k = 0.0;

                if(fd_squared>1e-12){
                    double numerator_k = (first_derivative.x() * second_derivative.y() - first_derivative.y() * second_derivative.x());
                    double denominator_k = std::pow(fd_squared, 1.5);  
                    curvature_k = numerator_k / denominator_k; 
                }

                spline_points_.push_back(position_s);
                spline_curvature_.push_back(curvature_k); 
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Generated %zu spline points and %zu curvature values. max_t=%.2f",
                spline_points_.size(),
                spline_curvature_.size(),
                max_t
            );

            /* TARGET VELOCITY PROFILE CALCULATION */
            // Compute the static profile once, right after receiving the track
            this->v_profile = utils::computeSmoothVel(this->spline_points_, this->spline_curvature_, 
                                                      this->config_, /*current_speed=*/0.0,
                                                      this->v_corner, this->v_accln, this->v_brake);

            // Cummulative S calculation
            this->s = utils::getCumulativeS(spline_points_);

            // Total loop length = distance to last point + distance from last point back to P0
            const std::vector<double> delta_s = utils::computeDeltaS(this->spline_points_);
            this->track_length = this->s.back() + delta_s.back();

            utils::saveProfileAnalysisToCSV(
                this->spline_points_,
                this->spline_curvature_,
                this->config_,
                this->v_profile,
                this->v_accln,
                this->v_brake,
                this->v_corner,
                "Bspline_velocity_profile_analysis.csv"
            );

            this->has_received_track = true;
        }); 

}

void Controller::controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg){

    // Check if the controller callback has received the track center points and then request for it 
    if (!has_received_track) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first track center points...");
        return;
    }

    double current_speed = static_cast<double>(msg->x_dot);
    double current_s = static_cast<double>(msg->s); 
    double current_pf = static_cast<double>(msg->performance_fraction);

    /* Setpoint generation */
    double current_progress = std::fmod(current_s, track_length);  // Helps to give the spline progression even after multiple laps

    // front, back is for values. begin and end gives iterators (memory pointers)
    auto it = std::upper_bound(s.begin(), s.end(), current_progress); // Search the sorted container s (from [first element] to (last)) 
    //and return an iterator pointing to the first element that is strictly greater than current_progress.

    size_t idx_next = 0;
    size_t idx_prev = 0;
    // this idx basically gives the index of 0 to 98 points that is stored in cummulative S in utils

    if (it == s.end()) { //
        // progress between the very last point and the first point (P0)
        idx_prev = s.size() - 1;
        idx_next = 0;
    } else if (it == s.begin()) {
        // progress is exactly at or before 0.0
        idx_prev = 0;
        idx_next = 1;
    } else {
        // progress falls cleanly between two track points
        idx_next = std::distance(s.begin(), it); // index distance (ex: 0 to 5 = 5 steps)
        idx_prev = idx_next - 1;
    } 

    // Linear interpolation for smooth velocity profile
    double s_prev = s[idx_prev];
    double s_next;

    if (idx_next == 0){
        // after 1 lap 
        s_next = track_length;
    }
    else{
        s_next = s[idx_next];
    }

    double t = (current_progress - s_prev) / (s_next - s_prev); 

    // v_profile vectors of prev and next indexes 
    double v_prev = v_profile[idx_prev];
    double v_next = v_profile[idx_next];

    // target acceleration for feedforward (a = v2 - u2 / 2s, assuming constant acceleration between every time instant)
    double s_del = s_next - s_prev; 
    double target_acceleration = (v_next * v_next - v_prev * v_prev) / (2 * s_del);
    
    // target velocity for pid
    double s_local = current_progress - s_prev;

    double target_velocity = std::sqrt(std::max(0.0, v_prev*v_prev + 2.0*target_acceleration*s_local));

    double u_ff = utils::computeFeedforward(target_acceleration, target_velocity, config_); 

    // PID Controller
    double u; 

    u = pid_1.calculateOutput(current_speed, target_velocity, u_ff); 

    std_msgs::msg::Float64 throttle_msg; 
    throttle_msg.data = u; 

    throttle_publisher->publish(throttle_msg); 
    
    double u_pid = pid_1.getPIDoutput(); 
    double u_unsaturated = pid_1.getUnsaturatedOutput();
    
    double u_p = pid_1.getPoutput();
    double u_i = pid_1.getIoutput();
    double u_d = pid_1.getDoutput();

    // Controller debug msg 
    lfs_msgs::msg::ControlDebug control_debug_msg; 

    control_debug_msg.header.stamp = this->get_clock()->now();
    control_debug_msg.u = u; 
    control_debug_msg.u_ff = u_ff;  
    control_debug_msg.u_pid = u_pid;  
    control_debug_msg.u_unsaturated = u_unsaturated;
    
    control_debug_msg.u_p = u_p; 
    control_debug_msg.u_i = u_i; 
    control_debug_msg.u_d = u_d; 

    control_debug_msg.v_target = target_velocity;  
    control_debug_msg.xdot = current_speed; 
    control_debug_msg.a_target = target_acceleration; 
    
    control_debug_msg.performance_fraction = current_pf; 

    control_debug_pub->publish(control_debug_msg); 

    // Debug logger
    RCLCPP_INFO(this->get_logger(), 
            "--- VERIFICATION --- Lap Progress: %.2fm | Match Index: [ %zu ] | Interp t: %.2f | Target V: %.2f m/s | Actual V: %.2f m/s | Feedforward: %.2f ", 
            current_progress, idx_prev, t, target_velocity, current_speed, u_ff);    
}