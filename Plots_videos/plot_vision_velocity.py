import rosbag2_py
import rclpy.serialization
from rosidl_runtime_py.utilities import get_message

import matplotlib.pyplot as plt
import numpy as np

bag_path = "/home/user/ros2_ws/src/vision_tracking"
topic_name = "/velocity_controller/commands"

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id="sqlite3"
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

msg_type = get_message(type_map[topic_name])

times = []
velocities = []

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic == topic_name:
        msg = rclpy.serialization.deserialize_message(
            data,
            msg_type
        )

        if len(msg.data) == 7:
            times.append(timestamp * 1e-9)
            velocities.append(msg.data)

times = np.array(times)
velocities = np.array(velocities)

# Start time from zero
times = times - times[0]

plt.figure(figsize=(10, 6))

for i in range(7):
    plt.plot(
        times,
        velocities[:, i],
        label=f"qdot{i+1}"
    )

plt.xlabel("Time [s]")
plt.ylabel("Commanded joint velocity [rad/s]")
plt.title("Vision Controller - Joint Velocity Commands")
plt.grid(True)
plt.legend()
plt.tight_layout()

output_path = "/home/user/ros2_ws/src/vision_velocity_commands.png"
plt.savefig(output_path, dpi=300)

print(f"Saved plot to: {output_path}")
print(f"Number of velocity samples: {len(times)}")

plt.show()
