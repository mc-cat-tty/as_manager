#pragma once
namespace signals::utils {
    
class LowPassFilter {

    private:
        float alpha;
        float filtered_value;

    public:
        LowPassFilter(float alpha_);
        void update(float z);
       float get_value();

};

}

