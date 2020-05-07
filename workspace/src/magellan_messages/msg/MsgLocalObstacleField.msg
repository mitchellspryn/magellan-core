Header header

# Real world grid bounds
geometry_msgs/Point32 grid_top_left_coords
geometry_msgs/Point32 grid_bottom_right_coords

# Number of cells in the grid
uint32 width
uint32 height

# Index into the grid, in row-major order
uint32[] cone_points
uint32[] cone_ids
uint32[] other_obstacle_point
uint32[] other_obstacle_ids
