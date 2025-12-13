import sys866_lib as ps
import P2_policy as P2
import Actor_Critic_policy as ac

NB_TEST = 20

env = ps.SimulinkInstance(
    sim_name='Simulation_simulink_Mac',
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=0.4, #gains de la politique P2
    init_Ki=0.04,
    init_Kd=0.0,
    dt_sim=0.1,
    pre_script='Simulation_matlab',
)

policy = ac.ActorCriticPolicy(
    consigne=None,
    consigne_tresh=0.97,
    loss_params={'error':0.95, 'dep': 20.0, 'conv': 0.05},
    dt=0.3
)

# policy = ps.Policy(
#     consigne=None,
#     consigne_tresh=0.97,
#     loss_params={'error':0.95, 'dep':20.0, 'conv':0.05},
#     dt=0.1 
# )

model_actor_path = 'models_ac/actor_model.pth'
model_critic_path = 'models_ac/critic_model.pth'
policy.load_actor_critic(model_actor_path, model_critic_path)


mean_running_loss_array = []
running_loss_array = []

#chrono pour la durée totale des tests  
start_time = ps.time.time()
for test in range(NB_TEST):
    print("\n" + "=" * 60)
    print(f" Test {test + 1} / {NB_TEST}")
    print("=" * 60)

    # Reset episode 
    policy.prev_error = 0.0
    policy.cumulative_error = 0.0
    policy.running_loss = 0.0
    policy.consigne = None

    episode_loop = ps.EpisodeLoop(
        env=env,
        consigne=None,
        policy=policy,
        max_steps=1000,
        max_time=45.0,
        policy_dt=0.1
    )
    
    running_loss, mean_running_loss = episode_loop.run_episode()
    running_loss_array.append(running_loss)
    mean_running_loss_array.append(mean_running_loss)

end_time = ps.time.time()
total_duration = end_time - start_time
mean_duration = total_duration / NB_TEST

print("\n" + "=" * 60)
print(" Résultats des tests ")
print("=" * 60)
overall_mean_running_loss = sum(mean_running_loss_array) / len(mean_running_loss_array)
overall_running_loss = sum(running_loss_array) / len(running_loss_array)
print(f" Perte totale moyenne sur les tests : {overall_running_loss:.4f}")
print(f" Perte moyenne par step sur les tests : {overall_mean_running_loss:.4f}")

print(f" Durée totale des tests : {total_duration:.2f} secondes")
print(f" Durée moyenne par test : {mean_duration:.2f} secondes")


env.eng.set_param(env.sim_name, 'SimulationCommand', 'stop', nargout=0)
env.close()