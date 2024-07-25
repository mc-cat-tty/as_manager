#include <signals/low_pass_filter.hpp>
#include <cassert>

namespace signals::utils {

LowPassFilter::LowPassFilter(float alpha_) : alpha(alpha_), filtered_value(0.0f) {
    assert(alpha >= 0.0f && alpha <= 1.0f);
}
        
void LowPassFilter::update(float z) {
    filtered_value = (1-alpha) * filtered_value + alpha * z;
}
float LowPassFilter::get_value() {
    return filtered_value;
}

}

