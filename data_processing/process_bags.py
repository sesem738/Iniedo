import os
import rosbag
import numpy as np

bag = rosbag.Bag('./data_processing/zed_trial.bag')

startTime = '1714671744.5199575'
endTime = '1714671873.66602'

# Read Data
raw_imu = [] #[ts wx wy wz ax ay az]

imu_topic = '/zed/zed_node/imu/data'

for (topic, msg, ts) in bag.read_messages():
    if topic == imu_topic:
        imu_i = np.array([
            msg.header.stamp.to_sec(),
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        raw_imu.append(imu_i)
    
raw_imu = np.asarray(raw_imu)

np.savetxt("imu_data", raw_imu, delimiter=",")
        