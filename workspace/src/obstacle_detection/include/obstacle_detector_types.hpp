#ifndef OBSTACLE_DETECTOR_TYPES_HPP
#define OBSTACLE_DETECTOR_TYPES_HPP

#include <cstdint>

typedef enum TraversabilityClassification
{
    SAFE=0,
    UNSAFE,
    UNSET
} TraversabilityClassification_t;

typedef struct StereoVisionPoint
{
    float x = 0;
    float y = 0;
    float z = 0;
    float nx = 0;
    float ny = 0;
    float nz = 0;
    float confidence = 0;
    uint32_t rgba_color = 0;
} StereoVisionPoint_t;

typedef struct StereoVisionPointMetadata
{
    TraversabilityClassification_t traversability = SAFE;
    int32_t cone_id = -1;
    bool is_valid = false;
} StereoVisionPointMetadata_t;

typedef struct PointAggregatorType
{
    int32_t unsafe_count = 0;
    int32_t safe_count = 0;
    int32_t cone_count = 0;
    int32_t cone_id = 0;
} PointAggregatorType_t;

#endif
