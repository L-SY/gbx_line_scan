//
// Low Pass Filter implementation for ROS2
//

#include "actuator_interface/common/lp_filter.h"
#include <cmath>

LowPassFilter::LowPassFilter(double cutoff_freq)
    : cutoff_freq_(cutoff_freq)
    , output_(0.0)
    , previous_input_(0.0)
    , previous_time_(rclcpp::Clock().now())
    , is_first_input_(true)
    , alpha_(0.0)
{
}

void LowPassFilter::input(double input_value, const rclcpp::Time& stamp) {
    if (is_first_input_) {
        output_ = input_value;
        previous_input_ = input_value;
        previous_time_ = stamp;
        is_first_input_ = false;
        return;
    }

    double dt = (stamp - previous_time_).seconds();
    if (dt > 0.0) {
        updateAlpha(dt);
        output_ = alpha_ * input_value + (1.0 - alpha_) * output_;
    }
    
    previous_input_ = input_value;
    previous_time_ = stamp;
}

double LowPassFilter::output() const {
    return output_;
}

void LowPassFilter::setCutoffFreq(double cutoff_freq) {
    cutoff_freq_ = cutoff_freq;
}

double LowPassFilter::getCutoffFreq() const {
    return cutoff_freq_;
}

void LowPassFilter::reset() {
    output_ = 0.0;
    previous_input_ = 0.0;
    previous_time_ = rclcpp::Clock().now();
    is_first_input_ = true;
    alpha_ = 0.0;
}

void LowPassFilter::updateAlpha(double dt) {
    if (dt > 0.0 && cutoff_freq_ > 0.0) {
        double rc = 1.0 / (2.0 * M_PI * cutoff_freq_);
        alpha_ = dt / (rc + dt);
    } else {
        alpha_ = 0.0;
    }
} 