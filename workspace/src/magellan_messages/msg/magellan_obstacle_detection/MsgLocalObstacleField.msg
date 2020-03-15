Header header
uint32 width
uint32 height

# Number of points per meter
# Assumed to be evenly distributed around center.
uint32 width_scale
uint32 height_scale

# 0 = no obstacle
# 1 = obstacle
# 2 = goal point
uint8[] occupancy_matrix
