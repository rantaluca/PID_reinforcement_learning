import sys866_lib_Windows as ps
#import sys866_lib_Mac as ps
from VFA_policy import RLSTDPolicy


instance = ps.SimulinkInstance(
    sim_name='Simulation_simulink_P2_Windows',
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=0.4,
    init_Ki=0.04,
    init_Kd=0.0,
    dt_sim=0.01,
    pre_script='Simulation_matlab_P2'
)

policy = ps.Policy(
    consigne=None,
    consigne_tresh=0.97,
    loss_params={'error':5.0, 'dep':0.5, 'conv':0.3},
    dt=0.1 
)

# policy = RLSTDPolicy(
#     consigne=None,
#     consigne_tresh=0.98,
#     loss_params={'error':0.5, 'dep':10, 'conv':0.5},
#     dt=0.1,
#     Kp_ref=1.0,
#     Ki_ref=0.1,
#     dKp=0.01,
#     dKi=0.001,
#     dKp2=0.05,
#     dKi2=0.005,
#     Kp_bounds=(0.0, 1.0),
#     Ki_bounds=(0.0, 1.0),
#     beta=100,
#     epsilon=0.1,
#     gamma=0.99,
#     Integral_ref=1.0,
#     Kp_init=0.4,
#     Ki_init=0.04,
# )





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