# autorally-traversability
Development on GT's small scale autonomous platform AutoRally, specifically on terrain traversability analysis

# How to run

1. Ensure you have pipenv installed:

```
pip install pipenv
```

2. in this directory (`/vip`) sync dependencies

```
pipenv sync
```

3. Run the labeling script
```
pipenv run python3 labeling.py
```

## bag files tested
Topics and camera intrinsics were found with `rosbag_dump.py` and updating the casename to each of the corresponding bagfiles.

### `alpha_autorally0_2020-07-23-16-27-57_0.bag`
- missing camera intrinsics matrix
- has weird state estimator issues with z
- Topic list:
```
['/RC/chassisCommand', '/autorally_core_manager/bond', '/camera/attn_image/compressed/parameter_descriptions', '/camera/attn_image/compressed/parameter_updates', '/camera/nn_cost', '/camera_nodelet_manager/bond', '/chassisState', '/diagnostics', '/gpsRoverStatus', '/imu/filter', '/imu/imu', '/imu/magnetic_field', '/imu/pressure', '/inner_boundary', '/lap_stats', '/left_CameraAutoBalance/histogram/compressed/parameter_descriptions', '/left_CameraAutoBalance/histogram/compressed/parameter_updates', '/left_CameraAutoBalance/parameter_descriptions', '/left_CameraAutoBalance/parameter_updates', '/left_CameraAutoBalance/roi/compressed/parameter_descriptions', '/left_CameraAutoBalance/roi/compressed/parameter_updates', '/left_camera/camera_info', '/left_camera/image_color/compressed', '/left_camera/image_color/compressed/parameter_descriptions', '/left_camera/image_color/compressed/parameter_updates', '/left_camera_nodelet/parameter_descriptions', '/left_camera_nodelet/parameter_updates', '/mppi_controller/chassisCommand', '/mppi_controller/feedbackPath', '/mppi_controller/mppiStatistics', '/mppi_controller/mppiStatus', '/mppi_controller/nominalPath', '/mppi_controller/nominalState', '/mppi_controller/parameter_descriptions', '/mppi_controller/parameter_updates', '/mppi_controller/stateDivergence', '/mppi_controller/subscribedPose', '/mppi_controller/timingInfo', '/outer_boundary', '/particle_filter/filter_attitude', '/particle_filter/filter_attitude_error', '/particle_filter/gps_bias', '/particle_filter/pose_estimate', '/rosout', '/rosout_agg', '/runstop', '/runstopBox', '/wheelSpeeds']
```
- Camera instrinsics:
``` 
[[0. 0. 0.]
 [0. 0. 0.]
 [0. 0. 0.]]

```
 
### `alpha_autorally3_2021-08-19-11-21-12.bag`
- still testing
- topic list: 
```
['/RC/chassisCommand', '/autorally_core_manager/bond', '/bias_acc', '/bias_gyro', '/camera/attn_image/compressed/parameter_descriptions', '/camera/attn_image/compressed/parameter_updates', '/camera/nn_cost', '/camera_nodelet_manager/bond', '/chassisState', '/diagnostics', '/gpsBaseRTCM3', '/gpsBaseRTCM3Xbee', '/gpsBaseStatus', '/gpsRoverStatus', '/imu/filter', '/imu/imu', '/imu/magnetic_field', '/imu/pressure', '/inner_boundary', '/left_CameraAutoBalance/histogram/compressed/parameter_descriptions', '/left_CameraAutoBalance/histogram/compressed/parameter_updates', '/left_CameraAutoBalance/parameter_descriptions', '/left_CameraAutoBalance/parameter_updates', '/left_CameraAutoBalance/roi/compressed/parameter_descriptions', '/left_CameraAutoBalance/roi/compressed/parameter_updates', '/left_camera/camera_info', '/left_camera/image_color/compressed', '/left_camera/image_color/compressed/parameter_descriptions', '/left_camera/image_color/compressed/parameter_updates', '/left_camera_nodelet/parameter_descriptions', '/left_camera_nodelet/parameter_updates', '/mppi_controller/chassisCommand', '/mppi_controller/feedbackPath', '/mppi_controller/mppiStatistics', '/mppi_controller/mppiStatus', '/mppi_controller/nominalPath', '/mppi_controller/nominalState', '/mppi_controller/parameter_descriptions', '/mppi_controller/parameter_updates', '/mppi_controller/stateDivergence', '/mppi_controller/subscribedPose', '/mppi_controller/timingInfo', '/outer_boundary', '/outer_bounday', '/pose_estimate', '/pose_estimate/status', '/rosout', '/rosout_agg', '/runstop', '/runstopBox', '/state_estimator/gps_pos', '/state_estimator/time_delays', '/wheelSpeeds', '/wheel_odom']
```
- Camera intrinsics:
```
[[840.187645   0.       639.5     ]
 [  0.       840.187645 511.5     ]
 [  0.         0.         1.      ]]
```


### `beta_autorally4_2021-02-21-18-30-44_0.bag`
- still testing
- missing camera instrinsics matrix
- topic list:
```
['/LTVMPC/chassisCommand', '/LTVMPC/parameter_descriptions', '/LTVMPC/parameter_updates', '/MAP_CA/lapStats', '/MAP_CA/mapCA', '/RC/chassisCommand', '/autorally_core_manager/bond', '/bias_acc', '/bias_gyro', '/chassisState', '/diagnostics', '/gpsBaseRTCM3', '/gpsBaseRTCM3Xbee', '/gpsBaseStatus', '/gpsRoverStatus', '/imu/filter', '/imu/imu', '/imu/magnetic_field', '/imu/pressure', '/pose_estimate', '/pose_estimate/status', '/rosout', '/runstop', '/runstopBox', '/state_estimator/time_delays', '/tf', '/wheelSpeeds', '/wheel_odom']
```

### `platformA_2015-08-13-12-02-41_split1.bag`
- still testing
- Topics:
```
['/RC/servoCommand', '/camera_nodelet_manager/bond', '/diagnostics', '/gpsRoverStatus', '/imu/filter', '/imu/imu', '/imu/magnetic_field', '/imu/pressure', '/infrastructure_manager/bond', '/left_camera/camera_info', '/left_camera/image_color', '/right_camera/camera_info', '/right_camera/image_color', '/rosout', '/rosout_agg', '/runstopSafeSeed', '/safeSpeed', '/servoStatus', '/tf', '/wheelSpeeds']
```
- camera intrinsics:
``` 
[849.19334598   0.         636.76166119]
 [  0.         849.52924899 505.65526302]
 [  0.           0.           1.        ]
 ```
 

