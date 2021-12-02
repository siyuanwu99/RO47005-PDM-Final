# Map server

Recommend to include cpp source files

Transfer point clouds to occupancy map using Raycast algorithm

For local map, you can subscribe `/map_generator/local_cloud`
For global map, you can subscribe `/map_generator/global_cloud`

Now it subscribe global map, and publish new 

## Run

first start the so3_simulator

Then, run following node with parameters
```
rosrun map_server test_grid_map_node /grid_map/x_size:=/random_forest/map/x_size /grid_map/y_size:=/random_forest/map/y_size /grid_map/z_size:=/random_forest/map/z_size ~cloud_in:=/map_generator/global_cloud 
```