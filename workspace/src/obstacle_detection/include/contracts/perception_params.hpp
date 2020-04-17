#ifndef PERCEPTION_PARAMS_HPP
#define PERCEPTION_PARAMS_HPP

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace magellan
{
    namespace obstacle_detection
    {
        class PerceptionParams
        {
            public:
                PerceptionParams() {}

                PerceptionParams(const PerceptionParams& other) :
                    _params(other._params) {};

                PerceptionParams(PerceptionParams&& other) noexcept :
                    _params(other._params) {};

                PerceptionParams& operator = (PerceptionParams other) 
                {
                    swap(*this, other);
                    return *this;
                }

                PerceptionParams& operator = (PerceptionParams&& other)
                {
                    swap(*this, other);
                    return *this;
                }

                friend void swap(PerceptionParams &a, PerceptionParams &b)
                {
                    using std::swap;
                    swap(a._params, b._params);
                }

                PerceptionParams(std::string file_name) { this->parse_params_from_file(file_name); }

                const std::unordered_map<std::string, std::string> &get_params() const { return this->_params; }
            private:
                std::unordered_map<std::string, std::string> _params;

                void parse_params_from_file(std::string file_name);
        };
    }
}

#endif
