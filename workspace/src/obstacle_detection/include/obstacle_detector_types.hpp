#ifndef OBSTACLE_DETECTOR_TYPES_HPP
#define OBSTACLE_DETECTOR_TYPES_HPP

#include <cstdint>

typedef struct HlsColor
{
    float h;
    float l;
    float s;
} HlsColor_t;

typedef enum TraversabilityClassification
{
    SAFE=0,
    UNSAFE,
    UNSET
} TraversabilityClassification_t;

typedef struct StereoVisionPoint
{
    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;
    float confidence;
    uint32_t rgba_color;
} StereoVisionPoint_t;

typedef struct StereoVisionPointMetadata
{
    TraversabilityClassification_t traversability;
    int32_t cone_id;
    bool is_valid;
} StereoVisionPointMetadata_t;

#endif
