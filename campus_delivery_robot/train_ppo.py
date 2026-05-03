import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from campus_delivery_robot.campus_env import CampusEnv

def main():
    env = CampusEnv()

    # 환경 검증
    print("환경 검증 중...")
    check_env(env, warn=True)
    print("환경 검증 완료!")

    # PPO 학습
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        tensorboard_log="./ppo_campus_log/"
    )

    print("학습 시작...")
    model.learn(total_timesteps=30_000)

    # 모델 저장
    model.save("ppo_campus_robot")
    print("모델 저장 완료: ppo_campus_robot.zip")

    env.close()

if __name__ == '__main__':
    main()
