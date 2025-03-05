# cartographer_detailed_comments_ws
 cartographer代码详解
 #### 常用命令
`rospack profile`：刷新ROS功能包；将当前ROS工作空间下的功能包放至ROS索引中
`rosbag info rslidar-outdoor-gps.bag`：了解bag中topic的名称与类型
`rosbag play XXX.bag`：运行bag包

#### lua文件解析
#### my_robot_2d_noImu.lua
```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                         -- 地图坐标系的名字
  tracking_frame = "base_link",              -- 将所有传感器数据转换到这个坐标系下	有IMU写imu_link,没有写base_link或footprint
  published_frame = "odom",                  -- tf: map -> odom	TF树最上面的节点
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
  --use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿

  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据
  
  lookup_transform_timeout_sec = 0.5,       -- 查找tf时的超时时间0.2
  submap_publish_period_sec = 0.1,          -- 发布数据的时间间隔0.3
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
--对之前包含的参数信息进行覆盖
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1



POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65


return options
```
##### trajectory_builder_2d.lua
```lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,            -- 是否使用imu数据
  min_range = 0.,                 -- 雷达数据的最远最近滤波, 保存中间值
  max_range = 30.,
  min_z = -0.8,                   -- 雷达数据的最高与最低的过滤, 保存中间值
  max_z = 2.,
  missing_data_ray_length = 5.,   -- 超过最大距离范围的数据点用这个距离代替
  num_accumulated_range_data = 1, -- 几帧有效的点云数据进行一次扫描匹配
  voxel_filter_size = 0.025,      -- 体素滤波的立方体的边长

  -- 使用固定的voxel滤波之后, 再使用自适应体素滤波器
  -- 体素滤波器 用于生成稀疏点云 以进行 扫描匹配
  adaptive_voxel_filter = {
    max_length = 0.5,             -- 尝试确定最佳的立方体边长, 边长最大为0.5
    min_num_points = 200,         -- 如果存在 较多点 并且大于'min_num_points', 则减小体素长度以尝试获得该最小点数
    max_range = 50.,              -- 距远离原点超过max_range 的点被移除
  },

  -- 闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  -- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
  -- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,             -- 线性搜索窗口的大小
    angular_search_window = math.rad(20.),  -- 角度搜索窗口的大小
    translation_delta_cost_weight = 1e-1,   -- 用于计算各部分score的权重
    rotation_delta_cost_weight = 1e-1,
  },

  -- 前端ceres匹配的一些配置参数
  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  -- 为了防止子图里插入太多数据, 在插入子图之前之前对数据进行过滤
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,

  -- 位姿预测器
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  -- 子图相关的一些配置
  submaps = {
    num_range_data = 90,          -- 一个子图里插入雷达数据的个数的一半
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID", -- 地图的种类, 还可以是tsdf格式
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      -- 概率占用栅格地图的一些配置
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      -- tsdf地图的一些配置
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
```

#### cartographer launch文件解析
```xml
<launch>

  <param name="/use_sim_time" value="fasle" />

  <node name="cartographer_node" pkg="cartographer_ros" type="cartographer_node" args="  
            -configuration_directory $(find cartographer_ros)/configuration_files  
            -configuration_basename my_robot_2d_noImu.lua" 
         output="screen">
    <remap from="scan" to="/scan" />
    <remap from="odom" to="/odometry/filtered"/>
  </node>
  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros" type="cartographer_occupancy_grid_node" />

  <node name="rviz" pkg="rviz" type="rviz" required="true" args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />

</launch>
```
## 学习思路

cartographer_ros是对cartographer的ROS封装，将数据格式转换并发布
cartographer部分源码主要分为了如图所示的几个文件夹：
	common：定义了基本数据结构和一些工具的使用接口：四舍五入取整的函数、时间转化相关的一些函数、数值计算的函数、互斥锁工具等
	sensor: 定义了传感器数据的相关数据结构。
	ransform: 定义了位姿的数据结构及其相关的转化。如2d\3d的刚体变换等
	mapping: 定义了上层应用的调用借口以及局部submap构建和基于闭环检测的位姿优化等的接口。
	io: 定义了一些与IO相关的工具，用于存读取数据、记录日志等

### CMakeList学习
**function.cmake**:定义函数，对传入的参数进行解析
### node_main.cc
### 刚体坐标变换
## Cartographer改进策略
### 体素滤波
### 回环检测
**Scan-to-Scan**：将相邻两帧之间的激光数据进行匹配，得到相对位姿
**Scan-to-Map**：
#### 延迟回环
当得到第一次回环检测时，不做处理，得到其相对位姿关系，第二次回环检测得到一个新的相对位姿关系，与第一个相差小于阈值，则证明回环是正确的，此时再进行回环
## Gmapping改进策略
### 去除运动畸变
有两种方法：基于传感器辅助的方法和ICP估计法
订阅里程计和激光帧数据，进行时间同步，求解激光帧中的点云对应的位姿数据，将位姿数据变换到相同坐标系下。
## C++11
```c++
  // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple
```
```c++
    // c++11: std::move 是将对象的状态或者所有权从一个对象转移到另一个对象, 
  // 只是转移, 没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能..
  // 右值引用是用来支持转移语义的.转移语义可以将资源 ( 堆, 系统对象等 ) 从一个对象转移到另一个对象, 
  // 这样能够减少不必要的临时对象的创建、拷贝以及销毁, 能够大幅度提高 C++ 应用程序的性能.
  // 临时对象的维护 ( 创建和销毁 ) 对性能有严重影响.
```
```
// c++11: =delete: 禁止编译器自动生成默认函数; =default: 要求编译器生成一个默认函数
```
```C++
  // c++11: map::emplace() 用于通过在容器中插入新元素来扩展map容器
  // 元素是直接构建的（既不复制也不移动）.仅当键不存在时才进行插入
  // c++11: std::forward_as_tuple tuple的完美转发
  // 该 tuple 在以右值为参数时拥有右值引用数据成员, 否则拥有左值引用数据成员
  // c++11: std::piecewise_construct 分次生成tuple的标志常量
  // 在map::emplace()中使用forward_as_tuple时必须要加piecewise_construct,不加就报错
  // https://www.cnblogs.com/guxuanqing/p/11396511.html
    extrapolators_.emplace(
      std::piecewise_construct, 
      std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}
```
### 智能指针
```c++
std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
```
`std::unique_ptr`
* 是 C++11 引入的一种智能指针，用于独占地管理动态分配的对象。
* 当 std::unique_ptr 被销毁时，它所管理的对象也会被自动销毁，避免了手动管理内存的复杂性和潜在的内存泄漏问题。
* std::unique_ptr 具有独占所有权，这意味着在任何时间点都只有一个 std::unique_ptr 可以拥有某个对象。
* **`std::unique_ptr`是一种独占所有权的智能指针，意味着在任意时刻，只有一个`std::unique_ptr`可以指向某个对象，不允许多个指针共享同一个对象的所有权**
* `std::unique_ptr` 提供独占所有权，`std::shared_ptr` 提供共享所有权，`std::weak_ptr `用于解决共享指针间的循环引用问题。

### template关键字
当要使用模板类中的模板函数时, 如果同时满足下面两个条件:
      1.如果模板类的模板参数是不确定类型时（int和非模板类等是确定类型）
      2.显式提供模板函数的模板参数, 不管模板参数是确定类型还是不确定类型
则，需要在模板函数前加template关键字

```c++
template <typename FloatType>
```
### explicit关键字

### =delete？？？

