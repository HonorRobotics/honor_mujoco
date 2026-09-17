
from __future__ import annotations

import threading

import mujoco
import numpy as np
import rclpy
from interaction_msgs.msg import LowCommand, LowState, MotorState
from sensor_msgs.msg import Imu

SENSORS_PER_MOTOR = 3
MODE_MACHINE = 0xFFFFFFFF
IMU_SENSOR_NAME = "imu_quat"

class Ros2Bridge:
    def __init__(self, mj_model, mj_data) -> None:
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = mj_model.nu
        self.dim_motor_sensor = SENSORS_PER_MOTOR * self.num_motor

        self.tau = np.zeros(self.num_motor)
        self.kp = np.zeros(self.num_motor)
        self.kd = np.zeros(self.num_motor)
        self.q = np.zeros(self.num_motor)
        self.dq = np.zeros(self.num_motor)

        # check sensor
        self._has_imu = False
        for index in range(self.dim_motor_sensor, mj_model.nsensor):
            name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_SENSOR, index)
            if name == IMU_SENSOR_NAME:
                self._has_imu = True

        # ros2
        rclpy.init()
        self._node = rclpy.create_node("ros2_bridge_node")

        self._low_state = LowState()
        self._imu = Imu()
        self._low_state_pub = self._node.create_publisher(LowState, "/xlab/hr/low_state", 10)
        self._imu_pub = self._node.create_publisher(Imu, "/xlab/hr/body_upper_imu", 10)
        self._low_cmd_sub = self._node.create_subscription(LowCommand, "/xlab/hr/low_cmd", self.low_cmd_handler, 10)
        self._timer = self._node.create_timer(0.003125, self._on_timer)

        self._spin_thread = threading.Thread(target=rclpy.spin,args = (self._node,))
        self._spin_thread.start()

    # input
    def low_cmd_handler(self, msg: LowCommand) -> None:
        if self.mj_data is None:
            return
        if len(msg.motor_cmd) != self.num_motor:
            print(
                "LowCmd motor_cmd size error!!!!!", len(msg.motor_cmd), self.num_motor
            )
            return

        for i, cmd in enumerate(msg.motor_cmd):
            self.tau[i] = cmd.tau
            self.kp[i] = cmd.kp
            self.kd[i] = cmd.kd
            self.q[i] = cmd.pos
            self.dq[i] = cmd.vel

    def update_torque(self) -> None:
        sensors = self.mj_data.sensordata
        for i in range(self.num_motor):
            position_error = self.q[i] - sensors[i]
            velocity_error = self.dq[i] - sensors[i + self.num_motor]
            self.mj_data.ctrl[i] = (
                self.tau[i] + self.kp[i] * position_error + self.kd[i] * velocity_error
            )

    # output
    def _on_timer(self) -> None:
        self._publish_low_state()
        self._publish_imu()

    def _publish_low_state(self) -> None:
        if self.mj_data is None:
            return

        sensors = self.mj_data.sensordata
        n = self.num_motor
        self._low_state.mode_machine = MODE_MACHINE
        self._low_state.motor_state = [MotorState() for _ in range(n)]
        for i in range(n):
            self._low_state.motor_state[i].pos_fb = sensors[i]
            self._low_state.motor_state[i].vel_fb = sensors[i + n]
            self._low_state.motor_state[i].tau_fb = sensors[i + 2 * n]

        self._low_state_pub.publish(self._low_state)

    def _publish_imu(self) -> None:
        if self.mj_data is not None and self._has_imu:

            sensors = self.mj_data.sensordata
            self._imu.orientation.w = sensors[self.dim_motor_sensor + 0]
            self._imu.orientation.x = sensors[self.dim_motor_sensor + 1]
            self._imu.orientation.y = sensors[self.dim_motor_sensor + 2]
            self._imu.orientation.z = sensors[self.dim_motor_sensor + 3]

            self._imu.angular_velocity.x = sensors[self.dim_motor_sensor + 4]
            self._imu.angular_velocity.y = sensors[self.dim_motor_sensor + 5]
            self._imu.angular_velocity.z = sensors[self.dim_motor_sensor + 6]

            self._imu.linear_acceleration.x = sensors[self.dim_motor_sensor + 7]
            self._imu.linear_acceleration.y = sensors[self.dim_motor_sensor + 8]
            self._imu.linear_acceleration.z = sensors[self.dim_motor_sensor + 9]

        self._imu_pub.publish(self._imu)

    # debug
    def print_scene_information(self) -> None:
        self._print_named_objects("Link", mujoco.mjtObj.mjOBJ_BODY, self.mj_model.nbody)
        self._print_named_objects(
            "Joint", mujoco.mjtObj.mjOBJ_JOINT, self.mj_model.njnt
        )
        self._print_named_objects(
            "Actuator", mujoco.mjtObj.mjOBJ_ACTUATOR, self.mj_model.nu
        )
        self._print_sensors()

    def _print_named_objects(self, label: str, obj_type: int, count: int) -> None:
        print(" ")
        print("<<------------- %s ------------->>" % label)
        for i in range(count):
            name = mujoco.mj_id2name(self.mj_model, obj_type, i)
            if name:
                print("%s_index:" % label.lower(), i, ", name:", name)

    def _print_sensors(self) -> None:
        print(" ")
        print("<<------------- Sensor ------------->>")
        offset = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            dim = self.mj_model.sensor_dim[i]
            if name:
                print("sensor_index:", offset, ", name:", name, ", dim:", dim)
            offset += dim
        print(" ")


class ElasticBand:

    KEY_SHORTEN  = 55   # glfw KEY_7
    KEY_LENGTHEN = 56   # glfw KEY_8
    KEY_TOGGLE   = 57   # glfw KEY_9
    LENGTH_STEP  = 0.1

    def __init__(self) -> None:
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def advance(self, x: np.ndarray, dx: np.ndarray) -> np.ndarray:
        offset = self.point - x
        distance = np.linalg.norm(offset)
        direction = offset / distance
        speed_along = np.dot(dx, direction)
        return (
            self.stiffness * (distance - self.length) - self.damping * speed_along
        ) * direction

    def key_callback(self, key: int) -> None:
        if key == self.KEY_SHORTEN:
            self.length -= self.LENGTH_STEP
        elif key == self.KEY_LENGTHEN:
            self.length += self.LENGTH_STEP
        elif key == self.KEY_TOGGLE:
            self.enable = not self.enable
