#ifndef GROUND_SEGMENTER_FACTORY_HPP
#define GROUND_SEGMENTER_FACTORY_HPP

#include <memory>
#include <unordered_map>
#include <stdexcept>

#include "ground_segmenter.hpp"
#include "planar_ransac_ground_segmenter.hpp"

class GroundSegmenterFactory
{
    public:
        std::unique_ptr<GroundSegmenter> create(const std::unordered_map<std::string, std::string> &parameters);
};

#endif
