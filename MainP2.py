import sys866_lib as ps


instance = ps.SimulinkInstance(
    sim_name='Simulation_SYS866',
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=6.0,
    init_Ki=2.5,
    init_Kd=1,
    dt_sim=0.01,
    pre_script='Projet_SYS866'
)

policy = ps.Policy(
    consigne=None,
    consigne_tresh=0.97,
    loss_params={'error':5.0, 'dep':0.5, 'conv':0.3},
    dt=0.1 
)

episode_loop = ps.EpisodeLoop(
    env=instance,
    consigne=None,
    policy=policy,
    max_steps=1000,
    policy_dt=0.1
)
episode_loop.run_episode()

instance.eng.set_param(instance.sim_name, 'SimulationCommand', 'stop', nargout=0)
instance.close()
