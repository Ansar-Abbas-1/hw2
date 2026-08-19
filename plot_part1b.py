import sqlite3
import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ============================================================
# Bag locations
# ============================================================

BAG_NORMAL = "../velocity_ctrl_bag/velocity_ctrl_bag_0.db3"
BAG_NULL = "../velocity_ctrl_null_bag/velocity_ctrl_null_bag_0.db3"


# ============================================================
# Joint limits
# Joints 1,3,5,7 -> +/- 2.96 rad
# Joints 2,4,6   -> +/- 2.09 rad
# ============================================================

LOWER_LIMITS = np.array([
    -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96
])

UPPER_LIMITS = np.array([
     2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96
])


# ============================================================
# Read one topic from rosbag sqlite3 database
# ============================================================

def read_topic(db_path, topic_name, msg_type):

    connection = sqlite3.connect(db_path)
    cursor = connection.cursor()

    cursor.execute(
        "SELECT id FROM topics WHERE name = ?",
        (topic_name,)
    )

    result = cursor.fetchone()

    if result is None:
        connection.close()
        raise RuntimeError(
            f"Topic {topic_name} not found in {db_path}"
        )

    topic_id = result[0]

    cursor.execute(
        """
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
        """,
        (topic_id,)
    )

    timestamps = []
    messages = []

    for timestamp, data in cursor.fetchall():

        msg = deserialize_message(
            data,
            msg_type
        )

        timestamps.append(
            timestamp * 1e-9
        )

        messages.append(msg)

    connection.close()

    return np.array(timestamps), messages


# ============================================================
# Load one experiment
# ============================================================

def load_experiment(db_path):

    # --------------------------------------------------------
    # Commanded joint velocities
    # --------------------------------------------------------

    t_cmd, cmd_msgs = read_topic(
        db_path,
        "/velocity_controller/commands",
        Float64MultiArray
    )

    qdot_cmd = np.array(
        [msg.data for msg in cmd_msgs],
        dtype=float
    )

    # --------------------------------------------------------
    # Measured joint positions
    # --------------------------------------------------------

    t_joint, joint_msgs = read_topic(
        db_path,
        "/joint_states",
        JointState
    )

    q = np.array(
        [msg.position[:7] for msg in joint_msgs],
        dtype=float
    )

    return (
        t_cmd,
        qdot_cmd,
        t_joint,
        q
    )


# ============================================================
# Automatically crop the useful motion interval
# ============================================================

def crop_active_period(
    t_cmd,
    qdot_cmd,
    t_joint,
    q
):

    cmd_norm = np.linalg.norm(
        qdot_cmd,
        axis=1
    )

    # Ignore very small numerical commands
    threshold = 1e-4

    active = np.where(
        cmd_norm > threshold
    )[0]

    if len(active) == 0:
        raise RuntimeError(
            "No active velocity commands found."
        )

    first = active[0]
    last = active[-1]

    start_time = t_cmd[first] - 0.2
    end_time = t_cmd[last] + 0.2

    cmd_mask = (
        (t_cmd >= start_time) &
        (t_cmd <= end_time)
    )

    joint_mask = (
        (t_joint >= start_time) &
        (t_joint <= end_time)
    )

    t_cmd_crop = (
        t_cmd[cmd_mask] - start_time
    )

    qdot_crop = qdot_cmd[cmd_mask]

    t_joint_crop = (
        t_joint[joint_mask] - start_time
    )

    q_crop = q[joint_mask]

    return (
        t_cmd_crop,
        qdot_crop,
        t_joint_crop,
        q_crop
    )


# ============================================================
# Plot commanded joint velocities
# ============================================================

def plot_velocities(
    t,
    qdot,
    title,
    filename
):

    plt.figure(
        figsize=(11, 7)
    )

    for i in range(7):

        plt.plot(
            t,
            qdot[:, i],
            linewidth=1.8,
            label=f"Joint {i+1}"
        )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Commanded joint velocity [rad/s]"
    )

    plt.title(title)

    plt.grid(True)

    plt.legend(
        ncol=2
    )

    plt.tight_layout()

    plt.savefig(
        filename,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    print(
        f"Saved: {filename}"
    )


# ============================================================
# Plot joint positions using one subplot per joint
# ============================================================

def plot_positions(
    t,
    q,
    title,
    filename
):

    fig, axes = plt.subplots(
        4,
        2,
        figsize=(12, 13),
        sharex=True
    )

    axes = axes.flatten()

    for i in range(7):

        ax = axes[i]

        # Joint position
        ax.plot(
            t,
            q[:, i],
            linewidth=2,
            label=f"Joint {i+1}"
        )

        # Correct upper limit for this joint
        ax.axhline(
            UPPER_LIMITS[i],
            linestyle="--",
            linewidth=1.5,
            label=f"Upper limit = {UPPER_LIMITS[i]:.2f} rad"
        )

        # Correct lower limit for this joint
        ax.axhline(
            LOWER_LIMITS[i],
            linestyle="--",
            linewidth=1.5,
            label=f"Lower limit = {LOWER_LIMITS[i]:.2f} rad"
        )

        ax.set_title(
            f"Joint {i+1}"
        )

        ax.set_ylabel(
            "Position [rad]"
        )

        ax.grid(True)

        ax.legend(
            loc="best",
            fontsize=8
        )

    # Last subplot is unused
    axes[7].axis("off")

    # X-axis labels
    axes[6].set_xlabel(
        "Time [s]"
    )

    fig.suptitle(
        title,
        fontsize=16
    )

    plt.tight_layout(
        rect=[0, 0, 1, 0.97]
    )

    plt.savefig(
        filename,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    print(
        f"Saved: {filename}"
    )


# ============================================================
# Direct Joint 4 comparison
# ============================================================

def plot_joint4_comparison(
    t_normal,
    q_normal,
    t_null,
    q_null,
    filename
):

    plt.figure(
        figsize=(10, 6)
    )

    plt.plot(
        t_normal,
        q_normal[:, 3],
        linewidth=2,
        label="velocity_ctrl"
    )

    plt.plot(
        t_null,
        q_null[:, 3],
        linewidth=2,
        label="velocity_ctrl_null"
    )

    plt.axhline(
        UPPER_LIMITS[3],
        linestyle="--",
        linewidth=1.5,
        label="Joint 4 upper limit = +2.09 rad"
    )

    plt.axhline(
        LOWER_LIMITS[3],
        linestyle="--",
        linewidth=1.5,
        label="Joint 4 lower limit = -2.09 rad"
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Joint 4 position [rad]"
    )

    plt.title(
        "Joint 4 Position Comparison"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        filename,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    print(
        f"Saved: {filename}"
    )


# ============================================================
# Print numerical joint-limit summary
# ============================================================

def print_limit_summary(
    q,
    controller_name
):

    print()
    print("=" * 65)
    print(controller_name)
    print("=" * 65)

    minimum_margin = float("inf")
    minimum_joint = None

    for i in range(7):

        q_min_observed = np.min(
            q[:, i]
        )

        q_max_observed = np.max(
            q[:, i]
        )

        distance_lower = (
            q_min_observed -
            LOWER_LIMITS[i]
        )

        distance_upper = (
            UPPER_LIMITS[i] -
            q_max_observed
        )

        nearest_margin = min(
            distance_lower,
            distance_upper
        )

        if nearest_margin < minimum_margin:

            minimum_margin = nearest_margin
            minimum_joint = i + 1

        print(
            f"Joint {i+1}: "
            f"min={q_min_observed:.4f}, "
            f"max={q_max_observed:.4f}, "
            f"nearest limit margin={nearest_margin:.4f} rad"
        )

    print()

    print(
        f"Minimum overall joint-limit margin: "
        f"{minimum_margin:.4f} rad "
        f"(Joint {minimum_joint})"
    )

    if minimum_margin < 0:

        print(
            "WARNING: a configured joint limit was exceeded."
        )

    else:

        print(
            "All joints remained inside the configured limits."
        )


# ============================================================
# Main
# ============================================================

print(
    "Loading velocity_ctrl bag..."
)

normal = load_experiment(
    BAG_NORMAL
)

print(
    "Loading velocity_ctrl_null bag..."
)

null = load_experiment(
    BAG_NULL
)


print(
    "Cropping active trajectory periods..."
)

normal_crop = crop_active_period(
    *normal
)

null_crop = crop_active_period(
    *null
)


(
    t_cmd_normal,
    qdot_normal,
    t_q_normal,
    q_normal
) = normal_crop


(
    t_cmd_null,
    qdot_null,
    t_q_null,
    q_null
) = null_crop


print()

print(
    f"velocity_ctrl active duration: "
    f"{t_cmd_normal[-1]:.3f} s"
)

print(
    f"velocity_ctrl_null active duration: "
    f"{t_cmd_null[-1]:.3f} s"
)


# ============================================================
# Required commanded velocity plots
# ============================================================

plot_velocities(
    t_cmd_normal,
    qdot_normal,
    "Commanded Joint Velocities - velocity_ctrl",
    "velocity_ctrl_commanded_velocities.png"
)

plot_velocities(
    t_cmd_null,
    qdot_null,
    "Commanded Joint Velocities - velocity_ctrl_null",
    "velocity_ctrl_null_commanded_velocities.png"
)


# ============================================================
# Improved joint position plots
# ============================================================

plot_positions(
    t_q_normal,
    q_normal,
    "Joint Positions - velocity_ctrl",
    "velocity_ctrl_joint_positions.png"
)

plot_positions(
    t_q_null,
    q_null,
    "Joint Positions - velocity_ctrl_null",
    "velocity_ctrl_null_joint_positions.png"
)


# ============================================================
# Extra direct comparison for Joint 4
# ============================================================

plot_joint4_comparison(
    t_q_normal,
    q_normal,
    t_q_null,
    q_null,
    "joint4_controller_comparison.png"
)


# ============================================================
# Numerical joint-limit comparison
# ============================================================

print_limit_summary(
    q_normal,
    "velocity_ctrl"
)

print_limit_summary(
    q_null,
    "velocity_ctrl_null"
)


# ============================================================
# Direct Joint 4 numerical comparison
# ============================================================

q4_max_normal = np.max(
    q_normal[:, 3]
)

q4_max_null = np.max(
    q_null[:, 3]
)

q4_upper_limit = UPPER_LIMITS[3]

margin_normal = (
    q4_upper_limit -
    q4_max_normal
)

margin_null = (
    q4_upper_limit -
    q4_max_null
)


print()
print("=" * 65)
print("JOINT 4 DIRECT COMPARISON")
print("=" * 65)

print(
    f"Joint 4 upper limit: "
    f"{q4_upper_limit:.4f} rad"
)

print(
    f"velocity_ctrl maximum q4: "
    f"{q4_max_normal:.4f} rad"
)

print(
    f"velocity_ctrl margin: "
    f"{margin_normal:.4f} rad"
)

print()

print(
    f"velocity_ctrl_null maximum q4: "
    f"{q4_max_null:.4f} rad"
)

print(
    f"velocity_ctrl_null margin: "
    f"{margin_null:.4f} rad"
)

print()

print(
    f"Improvement in Joint 4 limit margin: "
    f"{margin_null - margin_normal:.4f} rad"
)

print()
print("Done.")
