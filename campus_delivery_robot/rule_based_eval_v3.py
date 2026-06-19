import numpy as np
import rclpy
from campus_delivery_robot.campus_env_v3 import (
    CampusEnvV3, DELIVERY_POINTS, CHARGING_STATION, PICKUP_POINT
)

def rule_based_action(env):
    """
    규칙 기반 에이전트 v3:
    충전소가 외딴곳에 있어서 배터리 기준을 50%로 높임
    1. 배터리 50% 이하 → 충전소
    2. 들고 있는 배달 있으면 → 가장 가까운 배달지
    3. 손 비었으면 → 정문
    """
    if env.battery < 50:
        return 5  # 충전소 (v3에서는 action 5)

    carrying = [i for i in range(1, 5) if env.carrying[i]]
    if carrying:
        dists = {
            i: np.linalg.norm(env.robot_pos - DELIVERY_POINTS[i])
            for i in carrying
        }
        return min(dists, key=dists.get) - 1  # action index (0~3)

    return 4  # 정문 (v3에서는 action 4)

def evaluate(n_episodes=10):
    env = CampusEnvV3()

    total_delivered_list = []
    battery_deaths = 0
    unnecessary_charges = 0

    for ep in range(n_episodes):
        obs, _ = env.reset()
        ep_delivered = 0

        for step in range(30):
            action = rule_based_action(env)

            if action == 5 and env.battery > 80:
                unnecessary_charges += 1

            obs, reward, terminated, truncated, info = env.step(action)
            ep_delivered = info.get('total_delivered', ep_delivered)

            if terminated or truncated:
                break

        total_delivered_list.append(ep_delivered)
        if env.battery <= 0:
            battery_deaths += 1

        print(f"[Rule-v3] 에피소드 {ep+1:2d}: 배달={ep_delivered:2d}, 배터리={env.battery:.1f}%")

    print(f"\n=== Rule-Based v3 평가 결과 ({n_episodes}에피소드) ===")
    print(f"평균 배달 수:     {np.mean(total_delivered_list):.1f}")
    print(f"최대 배달 수:     {max(total_delivered_list)}")
    print(f"최소 배달 수:     {min(total_delivered_list)}")
    print(f"배터리 방전:      {battery_deaths}회")
    print(f"불필요한 충전:    {unnecessary_charges}회")

    env.close()

def main():
    rclpy.init()
    evaluate(n_episodes=10)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
