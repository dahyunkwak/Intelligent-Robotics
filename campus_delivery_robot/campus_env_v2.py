import gymnasium as gym
import numpy as np
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from sb3_contrib.common.wrappers import ActionMasker


# ─────────────────────────────────────────────
#  상수 정의 (v2: 배달지 6개)
# ─────────────────────────────────────────────
DELIVERY_POINTS = {
    1: np.array([ 12.0, -16.5]),   # ECC
    2: np.array([-12.0, -16.5]),   # 대강당
    3: np.array([-16.0,   3.0]),   # 학관
    4: np.array([ 16.0,   3.0]),   # 중앙도서관
    5: np.array([  0.0,  12.0]),   # 연구협력관
    6: np.array([ 18.0, -14.0]),   # 조형예술관
}
DELIVERY_NAMES = {
    1: "ECC",
    2: "대강당",
    3: "학관",
    4: "중앙도서관",
    5: "연구협력관",
    6: "조형예술관",
}
PICKUP_POINT     = np.array([0.0, -18.0])
CHARGING_STATION = np.array([0.0,   0.0])

MAX_STEPS               = 30
BATTERY_DRAIN_PER_METER = 0.5
BATTERY_CHARGE_AMOUNT   = 40.0
BATTERY_LOW_THRESHOLD   = 15.0
BATTERY_FAIL_PENALTY    = 10.0
COLLISION_DIST          = 0.5
GOAL_DIST               = 2.5
CHARGER_HALF_SIZE       = 3
NAV_RETRY_COUNT         = 3
NAV_RETRY_DELAY         = 3.0
NAV_TIMEOUT_SIM_SEC     = 120.0
CONSEC_FAIL_LIMIT       = 3
MAX_CARRY               = 2


# ─────────────────────────────────────────────
#  환경 클래스 (v2)
# ─────────────────────────────────────────────
class CampusEnvV2(gym.Env):
    """
    캠퍼스 배달 로봇 환경 v2 (배달지 6개)

    관측 (21차원):
        [0-1]   robot_pos (x, y)          [2]     battery
        [3-8]   delivered[1-6]            [9-14]  carrying[1-6]
        [15-20] dist_to_delivery[1-6]     [21]    dist_to_charger
        [22]    dist_to_pickup

    행동 (Discrete 8):
        0~5 → 배달지 1~6 로 이동 (carrying 중인 것만 허용)
        6   → 정문으로 이동   (손이 비었을 때만 허용)
        7   → 충전소로 이동   (항상 허용)
    """

    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()

        from rclpy.parameter import Parameter

        self.node = Node(
            'campus_env_v2_node',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.executor_thread.start()

        self._nav2_client = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose'
        )
        print("[ENV] 🧭 Nav2 Action Server 연결 대기 중...")
        start_time = time.time()
        connected  = threading.Event()

        def _check_connection():
            if self._nav2_client.wait_for_server(timeout_sec=60.0):
                connected.set()

        threading.Thread(target=_check_connection, daemon=True).start()

        while not connected.is_set():
            if time.time() - start_time > 65.0:
                raise RuntimeError("❌ Nav2 서버 연결 실패")
            print(f"[ENV] ⏳ 대기 중... ({time.time()-start_time:.0f}s)")
            time.sleep(0.5)

        print("[ENV] ✅ Nav2 Action Server 연결 성공!")

        self._states_lock = threading.Lock()

        self.model_states_sub = self.node.create_subscription(
            ModelStates, '/model_states', self._model_states_cb, 10
        )
        self.set_state_client = self.node.create_client(
            SetEntityState, '/set_entity_state'
        )
        self.initialpose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.clear_global_costmap_client = self.node.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.node.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        print("[ENV] ✅ 모든 클라이언트 / 퍼블리셔 생성 완료!")

        self.robot_pos       = PICKUP_POINT.copy()
        self.model_states    = None
        self.battery         = 100.0
        self.step_count      = 0
        self.delivered       = {i: False for i in range(1, 7)}
        self.carrying        = {i: False for i in range(1, 7)}
        self.total_delivered = 0
        self.consec_failures = 0
        self._last_nav_dist  = 0.0

        # action: 0~5=배달지1~6, 6=정문, 7=충전소
        self.action_space = gym.spaces.Discrete(8)

        # obs 23-dim
        self.observation_space = gym.spaces.Box(
            low=np.array(
                [-30,-30,  0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,  0],
                dtype=np.float32
            ),
            high=np.array(
                [ 30, 30,100,  1,1,1,1,1,1,  1,1,1,1,1,1, 80,80,80,80,80,80, 80, 80],
                dtype=np.float32
            ),
            dtype=np.float32
        )

    def _model_states_cb(self, msg):
        with self._states_lock:
            self.model_states = msg

    def _parse_states(self):
        with self._states_lock:
            states = self.model_states

        if states is None:
            return

        for i, name in enumerate(states.name):
            if name == 'turtlebot3_burger':
                pos = states.pose[i].position
                self.robot_pos = np.array([pos.x, pos.y])
                break

    def _teleport_robot(self, x: float, y: float) -> bool:
        if not self.set_state_client.wait_for_service(timeout_sec=30.0):
            print("[ENV] ❌ set_entity_state 서비스 없음 - 텔레포트 스킵")
            return False

        state = EntityState()
        state.name                   = 'turtlebot3_burger'
        state.pose.position.x        = float(x)
        state.pose.position.y        = float(y)
        state.pose.position.z        = 0.01
        state.pose.orientation.w     = 1.0
        state.twist                  = Twist()
        state.reference_frame        = 'world'

        req = SetEntityState.Request()
        req.state = state

        try:
            future  = self.set_state_client.call_async(req)
            timeout = time.time() + 15.0
            while not future.done():
                if time.time() > timeout:
                    print("[ENV] ⚠️ 텔레포트 타임아웃 (15s)")
                    return False
                time.sleep(0.05)
            print("[ENV] 🔄 로봇 위치 리셋 완료")
        except Exception as e:
            print(f"[ENV] ⚠️ 텔레포트 실패: {e}")
            return False

        time.sleep(1.0)
        self._reset_amcl(x, y)
        return True

    def _reset_amcl(self, x: float, y: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id         = 'map'
        msg.header.stamp            = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x    = float(x)
        msg.pose.pose.position.y    = float(y)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0]      = 0.25
        msg.pose.covariance[7]      = 0.25
        msg.pose.covariance[35]     = 0.07

        for _ in range(3):
            self.initialpose_pub.publish(msg)
            time.sleep(0.1)
        print(f"[ENV] 📍 AMCL 초기 위치 재설정: ({x:.1f}, {y:.1f})")

        for client in (
            self.clear_global_costmap_client,
            self.clear_local_costmap_client,
        ):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

        time.sleep(1.0)
        print("[ENV] 🗺️ Costmap 초기화 완료")

    def _clear_costmaps(self):
        for client in (self.clear_global_costmap_client, self.clear_local_costmap_client):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        time.sleep(0.5)

    def _navigate_to(self, x: float, y: float) -> bool:
        print(f"[NAV] 목표=({x:.1f}, {y:.1f})")

        self._clear_costmaps()

        goal = NavigateToPose.Goal()
        goal.pose                    = PoseStamped()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x    = float(x)
        goal.pose.pose.position.y    = float(y)
        goal.pose.pose.orientation.w = 1.0

        goal_handle = None
        for attempt in range(NAV_RETRY_COUNT):
            send_future = self._nav2_client.send_goal_async(goal)
            while not send_future.done():
                time.sleep(0.05)

            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break

            print(f"[NAV] ⚠️ 목표 거부됨 ({attempt+1}/{NAV_RETRY_COUNT}), "
                  f"{NAV_RETRY_DELAY}초 후 재시도...")
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            time.sleep(NAV_RETRY_DELAY)

        if goal_handle is None or not goal_handle.accepted:
            print("[NAV] ❌ 목표 거부됨 (재시도 모두 실패)")
            return False

        print("[NAV] ✅ 목표 수락됨, 이동 중...")
        result_future  = goal_handle.get_result_async()
        start_sim_time = self.node.get_clock().now()

        self._parse_states()
        _track_pos = self.robot_pos.copy()
        self._last_nav_dist = 0.0

        while not result_future.done():
            elapsed = (self.node.get_clock().now() - start_sim_time).nanoseconds / 1e9
            if elapsed > NAV_TIMEOUT_SIM_SEC:
                print(f"[NAV] ❌ 이동 타임아웃 (Sim {NAV_TIMEOUT_SIM_SEC}s)")
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)
            self._parse_states()
            self._last_nav_dist += float(np.linalg.norm(self.robot_pos - _track_pos))
            _track_pos = self.robot_pos.copy()

        status = result_future.result().status

        self._parse_states()
        self._last_nav_dist += float(np.linalg.norm(self.robot_pos - _track_pos))
        dist_to_goal = float(np.linalg.norm(
            self.robot_pos - np.array([x, y])
        ))

        if status == 4:
            print("[NAV] ✅ 도착 성공!")
            return True
        elif status == 6 and dist_to_goal < 2.0:
            print(f"[NAV] ⚠️ Status 6이지만 목표 근처 ({dist_to_goal:.1f}m) → 성공 처리")
            return True
        else:
            print(f"[NAV] ❌ 이동 실패 (Status: {status}, 거리: {dist_to_goal:.1f}m)")
            return False

    def action_masks(self) -> np.ndarray:
        active_carries = sum(self.carrying[i] for i in range(1, 7))
        hands_empty = active_carries == 0
        return np.array([
            self.carrying[1],   # 배달지1
            self.carrying[2],   # 배달지2
            self.carrying[3],   # 배달지3
            self.carrying[4],   # 배달지4
            self.carrying[5],   # 배달지5
            self.carrying[6],   # 배달지6
            hands_empty,        # 정문
            True,               # 충전소
        ], dtype=bool)

    def _get_obs(self) -> np.ndarray:
        dists        = [np.linalg.norm(self.robot_pos - DELIVERY_POINTS[i])
                        for i in range(1, 7)]
        dist_charger = np.linalg.norm(self.robot_pos - CHARGING_STATION)
        dist_pickup  = np.linalg.norm(self.robot_pos - PICKUP_POINT)

        return np.array([
            self.robot_pos[0],
            self.robot_pos[1],
            self.battery,
            float(self.delivered[1]),
            float(self.delivered[2]),
            float(self.delivered[3]),
            float(self.delivered[4]),
            float(self.delivered[5]),
            float(self.delivered[6]),
            float(self.carrying[1]),
            float(self.carrying[2]),
            float(self.carrying[3]),
            float(self.carrying[4]),
            float(self.carrying[5]),
            float(self.carrying[6]),
            *dists,
            dist_charger,
            dist_pickup,
        ], dtype=np.float32)

    def step(self, action: int):
        self.step_count += 1

        if action in range(6):
            target_id   = action + 1        # 1~6: 배달지
            target_pos  = DELIVERY_POINTS[target_id]
            target_name = DELIVERY_NAMES[target_id]
        elif action == 6:
            target_id   = 0                 # 정문
            target_pos  = PICKUP_POINT
            target_name = "정문"
        else:
            target_id   = -1                # 충전소
            target_pos  = CHARGING_STATION
            target_name = "충전소"

        carrying_now = [i for i in range(1, 7) if self.carrying[i]]
        print(f"\n[STEP {self.step_count}] action={action} → {target_name} "
              f"({target_pos[0]:.1f}, {target_pos[1]:.1f}) | "
              f"배터리={self.battery:.1f} | 배달중={carrying_now} | 누적={self.total_delivered}")

        nav_success = self._navigate_to(target_pos[0], target_pos[1])

        time.sleep(1.0)
        self._parse_states()

        dist_traveled = self._last_nav_dist
        self.battery -= dist_traveled * BATTERY_DRAIN_PER_METER

        if not nav_success:
            self.battery -= BATTERY_FAIL_PENALTY

        self.battery = max(0.0, self.battery)

        if target_id in range(1, 7):
            if nav_success:
                self.consec_failures = 0
            else:
                self.consec_failures += 1

        dist_to_target = np.linalg.norm(self.robot_pos - target_pos)

        reward     = 0.0
        terminated = False

        if not nav_success:
            reward -= 5.0

        reward -= dist_traveled * 0.3

        # 배달지 도착
        if target_id in range(1, 7) and dist_to_target < GOAL_DIST:
            if self.carrying[target_id]:
                reward += 100.0
                self.delivered[target_id] = True
                self.carrying[target_id]  = False
                self.total_delivered      += 1
                print(f"📦 배달 완료: {DELIVERY_NAMES[target_id]} (누적={self.total_delivered})")

        # 정문 도착 → 새 배치 픽업
        if target_id == 0 and dist_to_target < GOAL_DIST:
            active_now = sum(self.carrying[i] for i in range(1, 7))
            if active_now == 0:
                new_order = list(range(1, 7))
                np.random.shuffle(new_order)
                new_batch = new_order[:MAX_CARRY]
                for d in range(1, 7):
                    self.delivered[d] = False
                for d in new_batch:
                    self.carrying[d] = True
                print(f"📥 정문 픽업: {new_batch}")
            else:
                reward -= 10.0
                print("⚠️ 불필요한 정문 방문")

        # 충전소 도착
        at_charger = (
            abs(self.robot_pos[0] - CHARGING_STATION[0]) < CHARGER_HALF_SIZE and
            abs(self.robot_pos[1] - CHARGING_STATION[1]) < CHARGER_HALF_SIZE
        )
        if target_id == -1 and at_charger:
            charged = min(BATTERY_CHARGE_AMOUNT, 100.0 - self.battery)
            self.battery += charged
            reward += charged * 0.3 - (100.0 - charged) * 0.05
            print(f"🔋 충전: +{charged:.1f} → 보상 {charged*0.3-(100.0-charged)*0.05:.1f}")

        # 배터리 방전
        if self.battery <= 0:
            reward    -= 150.0
            terminated = True
            print("🪫 배터리 방전!")

        if self.consec_failures >= CONSEC_FAIL_LIMIT:
            reward    -= 30.0
            terminated = True
            print(f"🚨 연속 {self.consec_failures}회 배달지 nav 실패 → 에피소드 종료")

        truncated = self.step_count >= MAX_STEPS
        obs       = self._get_obs()

        return obs, reward, terminated, truncated, {'total_delivered': self.total_delivered, 'battery_dead': self.battery <= 0}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count      = 0
        self.battery         = 100.0
        self.delivered       = {i: False for i in range(1, 7)}
        self.carrying        = {i: False for i in range(1, 7)}
        self.total_delivered = 0
        self.consec_failures = 0

        order = list(range(1, 7))
        np.random.shuffle(order)
        first_batch = order[:MAX_CARRY]
        for d in first_batch:
            self.carrying[d] = True
        print(f"[ENV] 📋 에피소드 시작 | 첫 배달={first_batch}")

        success = self._teleport_robot(PICKUP_POINT[0], PICKUP_POINT[1])
        if not success:
            print("[ENV] ⚠️ 텔레포트 실패 - 현재 위치에서 에피소드 시작")

        time.sleep(1.0)
        self._parse_states()

        return self._get_obs(), {}

    def close(self):
        if hasattr(self, 'executor'):
            self.executor.shutdown()
        if hasattr(self, 'executor_thread') and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)
        if hasattr(self, '_nav2_client'):
            self._nav2_client.destroy()
        if hasattr(self, 'model_states_sub'):
            self.node.destroy_subscription(self.model_states_sub)
        if hasattr(self, 'set_state_client'):
            self.node.destroy_client(self.set_state_client)
        if hasattr(self, 'node'):
            self.node.destroy_node()
            print("🤖 ROS 2 Node destroyed.")
        if rclpy.ok():
            rclpy.shutdown()
            print("🛑 rclpy shutdown complete.")


if __name__ == '__main__':
    env = DummyVecEnv([lambda: ActionMasker(CampusEnvV2(), lambda e: e.action_masks())])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)
    print("✅ 환경 v2 세팅 완료")
