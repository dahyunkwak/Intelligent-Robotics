import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty

# 목표 위치 정의
DELIVERY_POINTS = {
    1: np.array([6.0, 5.0]),
    2: np.array([-6.0, 5.0]),
}
CHARGING_STATION = np.array([0.0, 0.0])

# 하이퍼파라미터
MAX_STEPS = 1000
BATTERY_DRAIN_PER_STEP = 0.05   # 스텝마다 배터리 소모
BATTERY_CHARGE_PER_STEP = 5.0  # 충전소에서 충전량
COLLISION_DIST = 0.5            # 보행자 충돌 거리
NEAR_DIST = 1.5                 # 보행자 근접 경고 거리
GOAL_DIST = 1.0                 # 목표 도달 거리


class CampusEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # ROS2 초기화
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('campus_env_node')

        # Publisher / Subscriber
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.model_states_sub = self.node.create_subscription(
            ModelStates, '/model_states', self._model_states_cb, 10
        )

        # Gazebo reset 서비스
        self.reset_client = self.node.create_client(Empty, '/gazebo/reset_simulation')

        # 내부 상태
        self.robot_pos = np.array([0.0, 0.0])
        self.robot_vel = np.array([0.0, 0.0])
        self.pedestrian_positions = []
        self.model_states = None

        # 배터리
        self.battery = 100.0

        # 현재 목표 (1 or 2 = 배달지, 0 = 충전소)
        self.current_target = 1

        # 스텝 카운터
        self.step_count = 0

        # Action space: 0 = 배달 계속, 1 = 충전소 이동
        self.action_space = gym.spaces.Discrete(2)

        # State space:
        # [로봇x, 로봇y, 목표x, 목표y, 배터리,
        #  가장 가까운 보행자까지 거리, 보행자 수(근접)]
        self.observation_space = gym.spaces.Box(
            low=np.array([-25, -25, -25, -25, 0, 0, 0], dtype=np.float32),
            high=np.array([25,  25,  25,  25, 100, 30, 10], dtype=np.float32),
            dtype=np.float32
        )

    def _model_states_cb(self, msg):
        self.model_states = msg

    def _get_model_states(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def _parse_states(self):
        if self.model_states is None:
            return

        for i, name in enumerate(self.model_states.name):
            pos = self.model_states.pose[i].position

            if name == 'turtlebot3_burger':
                self.robot_pos = np.array([pos.x, pos.y])
                twist = self.model_states.twist[i]
                self.robot_vel = np.array([twist.linear.x, twist.linear.y])

            elif 'pedestrian' in name:
                self.pedestrian_positions.append(np.array([pos.x, pos.y]))

    def _get_obs(self):
        target_pos = DELIVERY_POINTS[self.current_target] \
            if self.current_target != 0 else CHARGING_STATION

        # 보행자 거리 계산
        if self.pedestrian_positions:
            dists = [np.linalg.norm(self.robot_pos - p)
                     for p in self.pedestrian_positions]
            min_dist = min(dists)
            near_count = sum(1 for d in dists if d < NEAR_DIST)
        else:
            min_dist = 30.0
            near_count = 0

        return np.array([
            self.robot_pos[0],
            self.robot_pos[1],
            target_pos[0],
            target_pos[1],
            self.battery,
            min_dist,
            float(near_count)
        ], dtype=np.float32)

    def _send_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _move_toward_target(self, target_pos):
        """목표 방향으로 간단한 proportional control"""
        diff = target_pos - self.robot_pos
        dist = np.linalg.norm(diff)
        angle = np.arctan2(diff[1], diff[0])

        linear = min(0.3, dist * 0.3)
        angular = np.clip(angle * 0.5, -1.0, 1.0)
        self._send_vel(linear, angular)
        return dist

    def step(self, action):
        self.step_count += 1
        self.pedestrian_positions = []

        # 상태 업데이트
        self._get_model_states()
        self._parse_states()

        # action 0: 배달 계속, action 1: 충전소로
        if action == 1:
            self.current_target = 0  # 충전소
        else:
            if self.current_target == 0:
                self.current_target = 1  # 배달 재개

        # 목표 위치
        if self.current_target == 0:
            target_pos = CHARGING_STATION
        else:
            target_pos = DELIVERY_POINTS[self.current_target]

        dist_to_target = self._move_toward_target(target_pos)

        # 배터리 소모
        self.battery -= BATTERY_DRAIN_PER_STEP
        self.battery = max(0.0, self.battery)

        # 충전소 도달 시 충전
        dist_to_charger = np.linalg.norm(self.robot_pos - CHARGING_STATION)
        if dist_to_charger < GOAL_DIST:
            self.battery = min(100.0, self.battery + BATTERY_CHARGE_PER_STEP)

        # 보행자 거리
        if self.pedestrian_positions:
            dists = [np.linalg.norm(self.robot_pos - p)
                     for p in self.pedestrian_positions]
            min_ped_dist = min(dists)
        else:
            min_ped_dist = 30.0

        # ========== 보상 함수 ==========
        reward = 0.0
        terminated = False

        # 1. 스텝 패널티 (최단 경로 유도)
        reward -= 0.1

        # 2. 목표 도달
        if dist_to_target < GOAL_DIST:
            if self.current_target != 0:
                reward += 100.0
                terminated = True
            else:
                reward += 20.0  # 충전 완료 보상
                self.current_target = 1  # 배달 재개

        # 3. 보행자 충돌
        if min_ped_dist < COLLISION_DIST:
            reward -= 100.0
            terminated = True

        # 4. 보행자 근접 패널티
        elif min_ped_dist < NEAR_DIST:
            reward -= (NEAR_DIST - min_ped_dist) * 2.0

        # 5. 배터리 방전
        if self.battery <= 0:
            reward -= 100.0
            terminated = True

        # 6. 배터리 낮을 때 충전소 안 가면 패널티
        if self.battery < 20.0 and action == 0:
            reward -= 1.0

        truncated = self.step_count >= MAX_STEPS
        obs = self._get_obs()

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.battery = 100.0
        self.current_target = 1
        self.pedestrian_positions = []

        # Gazebo 리셋
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.reset_client.call_async(Empty.Request())

        self._get_model_states()
        self._parse_states()

        return self._get_obs(), {}

    def close(self):
        self._send_vel(0.0, 0.0)
        self.node.destroy_node()
