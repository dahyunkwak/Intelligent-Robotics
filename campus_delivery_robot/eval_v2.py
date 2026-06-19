import numpy as np
import rclpy
from sb3_contrib import MaskablePPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from campus_delivery_robot.campus_env_v2 import CampusEnvV2
from sb3_contrib.common.wrappers import ActionMasker

def mask_fn(env):
    return env.action_masks()

def make_env():
    env = CampusEnvV2()
    env = ActionMasker(env, mask_fn)
    return env

def evaluate(n_episodes=10):
    env = DummyVecEnv([make_env])
    env = VecNormalize.load(
        "/home/dahyun/ros2_ws/src/campus_delivery_robot/ppo_campus_vecnormalize_v2.pkl",
        env
    )
    env.training = False
    env.norm_reward = False

    model = MaskablePPO.load(
        "/home/dahyun/ros2_ws/src/campus_delivery_robot/ppo_campus_robot_v2",
        env=env
    )

    total_delivered_list = []
    battery_deaths = 0
    unnecessary_charges = 0

    for ep in range(n_episodes):
        obs = env.reset()
        ep_delivered = 0

        for step in range(30):
            action_masks = env.env_method("action_masks")
            action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
            obs, reward, done, info = env.step(action)

            ep_delivered = info[0].get('total_delivered', ep_delivered)

            raw_env = env.envs[0].env
            if action[0] == 7 and raw_env.battery > 80:
                unnecessary_charges += 1

            if info[0].get('battery_dead', False):
                battery_deaths += 1

            if done[0]:
                break

        total_delivered_list.append(ep_delivered)
        raw_env = env.envs[0].env
        print(f"[RL-v2] 에피소드 {ep+1:2d}: 배달={ep_delivered:2d}, 배터리={raw_env.battery:.1f}%")

    print(f"\n=== RL v2 모델 평가 결과 ({n_episodes}에피소드) ===")
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
