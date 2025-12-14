import sys866_lib_Mac as ps
# si windows utilisez la ligne suivante à la place de la précédente
# import sys866_lib_windows as ps
import Actor_Critic_policy as ac
import torch
import os
import time


# Config

N_EPOCHS = 40
MAX_STEPS = 1000
POLICY_DT = 0.3
DT_SIM = 0.1
MAX_TIME = 30 #30 secondes

GUI = False        # True = matplotlib / False = headless
SAVE_EVERY = 5    # Sauvegarder les modèles tous les n epochs

MODEL_DIR = "models_ac"
os.makedirs(MODEL_DIR, exist_ok=True)


# Environment setup
env = ps.SimulinkInstance(
    sim_name='Simulation_simulink_Mac',
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=0.4, #gains de la politique P2
    init_Ki=0.04,
    init_Kd=0.0,
    dt_sim=DT_SIM,
    pre_script='Simulation_matlab',
)

# Policy setup
policy = ac.ActorCriticPolicy(
    consigne=None,
    consigne_tresh=0.97,
    loss_params={'error':0.95, 'dep': 20.0, 'conv': 0.05},
    Kp_init=0.4,
    Ki_init=0.04,
    dt=POLICY_DT
)

episode = ac.EpisodeLoopActorCritic(
    env=env,
    consigne=None,
    policy=policy,
    max_steps=MAX_STEPS,
    max_time=MAX_TIME,
    policy_dt=POLICY_DT,
    plot=GUI
)

# Train Loop
for epoch in range(N_EPOCHS):

    print("\n" + "=" * 60)
    print(f" Epoch {epoch + 1} / {N_EPOCHS}")
    print("=" * 60)

    # Reset episode 
    policy.prev_error = 0.0
    policy.cumulative_error = 0.0
    policy.running_loss = 0.0
    policy.consigne = None

    # Run episode
    episode.run_episode()

    print(f"Running loss: {policy.running_loss:.4f}")

    # Save models periodically
    if (epoch + 1) % SAVE_EVERY == 0:
        actor_path = os.path.join(MODEL_DIR, f"actor_model_epoch_{epoch + 1}.pth")
        critic_path = os.path.join(MODEL_DIR, f"critic_model_epoch_{epoch + 1}.pth")
        policy.save_actor_critic(actor_path, critic_path)
        print(f"Models saved at epoch {epoch + 1} to {MODEL_DIR}")

print("\n --- Fin du training --- \n")
env.eng.set_param(env.sim_name, 'SimulationCommand', 'stop', nargout=0)
env.close()
