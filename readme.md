# benchmark_pipelines
A package to compare MoveIt! pipelines

## Usage dependencies

The package assumes that you have a ROS/MoveIt! robot running.
If you have a properly set-up moveit config package, the following should suffice:
```
roslaunch {your_robot}_moveit_config demo.launch
```

## How to use

To run the benchmarking:

1. Define properties in ```config/multigoal_queries.yaml```
2. ```roslaunch benchmark_pipelines execute_multigoal_benchmark```. By default, results are saved in a CSV file under your .ros folder.
3. Adjust plot options in ```config/plot.yaml```. Make sure you set the path and the CSV file name properly.
4. ```roslaunch benchmark_pipelines analyze_results```. By default, images are saved under your .ros folder.


