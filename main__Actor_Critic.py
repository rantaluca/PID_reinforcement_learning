import sys866_lib_Mac as ps
# si windows utilisez la ligne suivante à la place de la précédente
# import sys866_lib_windows as ps
import Actor_Critic_policy as ac

env = ps.SimulinkInstance(
    sim_name='Simulation_simulink_Mac',
    # sim_name='Simulation_simulink_Windows', # si windows decommentez cette ligne et commentez la ligne au dessus
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=0.4, #gains de la politique P2
    init_Ki=0.04,
    init_Kd=0.0,
    dt_sim=0.01,
    pre_script='Simulation_matlab',
)

policy = ac.ActorCriticPolicy(
    consigne=None,
    consigne_tresh=0.97,
    loss_params={'error':0.95, 'dep': 20.0, 'conv': 0.05},
    dt=0.1,
)

model_actor_path = 'models_ac/actor_model_epoch_old_15.pth'
model_critic_path = 'models_ac/critic_model_epoch_old_15.pth'
policy.load_actor_critic(model_actor_path, model_critic_path)

episode_loop = ps.EpisodeLoop(
    env=env,
    consigne=None,
    policy=policy,
    max_steps=1000,
    max_time=100.0,
    policy_dt=0.1
)


episode_loop.run_episode()

env.eng.set_param(env.sim_name, 'SimulationCommand', 'stop', nargout=0)
env.close()