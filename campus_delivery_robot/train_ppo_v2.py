import os
from sb3_contrib import MaskablePPO
from sb3_contrib.common.wrappers import ActionMasker
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from campus_delivery_robot.campus_env_v2 import CampusEnvV2


class DeliveryLogCallback(BaseCallback):
    """에피소드당 총 배달 수를 TensorBoard에 기록."""

    def __init__(self):
        super().__init__()
        self._ep_delivered = 0

    def _on_step(self) -> bool:
        info = self.locals.get("infos", [{}])[0]
        if "total_delivered" in info:
            self._ep_delivered = info["total_delivered"]
        if self.locals.get("dones", [False])[0]:
            self.logger.record("delivery/ep_total_delivered", self._ep_delivered)
            self._ep_delivered = 0
        return True


# ─────────────────────────────────────────────────────────────
#  하이퍼파라미터
# ─────────────────────────────────────────────────────────────
CFG = dict(
    total_timesteps = 20_000,
    learning_rate   = 3e-4,
    n_steps         = 128,
    batch_size      = 64,
    n_epochs        = 10,
    gamma           = 0.93,
    ent_coef        = 0.02,
    max_grad_norm   = 0.5,
    net_arch        = [128, 128],
    checkpoint_freq = 2048,
    log_dir         = "./ppo_campus_log_v2/",
    ckpt_dir        = "./checkpoints_v2/",
    model_path      = "ppo_campus_robot_v2",
    vecnorm_path    = "ppo_campus_vecnormalize_v2.pkl",
)


# ─────────────────────────────────────────────────────────────
#  환경 팩토리
# ─────────────────────────────────────────────────────────────
def make_env():
    def _init():
        env = CampusEnvV2()
        env = ActionMasker(env, lambda e: e.action_masks())
        return env
    return _init


# ─────────────────────────────────────────────────────────────
#  저장 헬퍼
# ─────────────────────────────────────────────────────────────
def save_all(model: MaskablePPO, env: VecNormalize):
    model.save(CFG["model_path"])
    env.save(CFG["vecnorm_path"])
    print(
        f"💾 저장 완료\n"
        f"   모델   : {CFG['model_path']}.zip\n"
        f"   VecNorm: {CFG['vecnorm_path']}"
    )


# ─────────────────────────────────────────────────────────────
#  메인
# ─────────────────────────────────────────────────────────────
def main():
    print("🌏 환경 v2 초기화 중...")
    env = DummyVecEnv([make_env()])
    env = VecNormalize(
        env,
        norm_obs    = True,
        norm_reward = True,
        clip_obs    = 20.0,
    )

    checkpoint_cb  = CheckpointCallback(
        save_freq         = CFG["checkpoint_freq"],
        save_path         = CFG["ckpt_dir"],
        name_prefix       = "ppo_campus_v2",
        save_vecnormalize = True,
    )
    delivery_log_cb = DeliveryLogCallback()

    if os.path.exists(CFG["model_path"] + ".zip"):
        print("📂 기존 v2 모델 로드 중...")
        env = VecNormalize.load(CFG["vecnorm_path"], env.venv)
        env.training = True
        env.norm_reward = True
        model = MaskablePPO.load(CFG["model_path"], env=env)
    else:
        print("🆕 새 v2 모델 생성")
        model = MaskablePPO(
            "MlpPolicy",
            env,
            verbose         = 1,
            learning_rate   = CFG["learning_rate"],
            n_steps         = CFG["n_steps"],
            batch_size      = CFG["batch_size"],
            n_epochs        = CFG["n_epochs"],
            gamma           = CFG["gamma"],
            ent_coef        = CFG["ent_coef"],
            max_grad_norm   = CFG["max_grad_norm"],
            policy_kwargs   = dict(net_arch=CFG["net_arch"]),
            tensorboard_log = CFG["log_dir"],
        )

    print(f"🚀 강화학습 v2 시작 (총 {CFG['total_timesteps']:,} 스텝)")

    try:
        model.learn(
            total_timesteps     = CFG["total_timesteps"],
            callback            = [checkpoint_cb, delivery_log_cb],
            progress_bar        = True,
            reset_num_timesteps = False,
        )
        print("✅ 학습 완료!")

    except KeyboardInterrupt:
        print("\n🛑 학습 중단 (Ctrl+C).")

    except Exception as e:
        print(f"\n❌ 예외 발생: {type(e).__name__}: {e}")

    finally:
        save_all(model, env)
        env.close()
        print("🏁 프로세스 안전 종료 완료.")


if __name__ == '__main__':
    main()
