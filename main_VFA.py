import sys866_lib_Windows as ps
#import sys866_lib_Mac as ps
from VFA_policy import RLSTDPolicy
import time
import os


instance = ps.SimulinkInstance(
    sim_name='Simulation_simulink_Windows',
    pid_block='PID_Controller',
    consigne_block=None,
    init_Kp=0.4,
    init_Ki=0.04,
    init_Kd=0.0,
    dt_sim=0.01,
    pre_script='Simulation_matlab'
)


policy = RLSTDPolicy(
    consigne=None,
    consigne_tresh=0.98,
    loss_params={'error':0.5, 'dep':10, 'conv':0.5},
    dt=0.1,
    Kp_ref=1.0,
    Ki_ref=0.1,
    dKp=0.01,
    dKi=0.001,
    dKp2=0.05,
    dKi2=0.005,
    Kp_bounds=(0.0, 1.0),
    Ki_bounds=(0.0, 1.0),
    beta=100,
    epsilon=0.1,
    gamma=0.99,
    Integral_ref=1.0,
    Kp_init=0.4,
    Ki_init=0.04,
)




#############
# Protocole :

    # Phase 1 : RLSTD s'entraine sur gains fixes (P2)
        # train_value_function = True
        # adapt_gains = False
    
    # Phase 2 : RLSTD s'entraine avec politique epsilon-greedy
        # train_value_function = True
        # adapt_gains = True
        # epsilon = 1 ---> 0
        # Hyperparamètres à définir :
            # action_sets = 0 ou 1 (à voir)
            # diminution d'epsilon avec le temps

    # Phase 3 : RLSTD est testé avec politique greedy
        # train_value_function = False
        # adapt_gains = False
        # epsilon = 0
        # Hyperparamètres à définir :
            # action_sets = 0 ou 1 (à voir)



#############            
##### Phase 1 

#policy.train_value_function = True
#policy.adapt_gains = False

# episode_loop = ps.EpisodeLoop(
#     env=instance,
#     consigne=None,
#     policy=policy,
#     max_steps=3000,
#     policy_dt=0.1,
#     max_sim_time=30.0
# )

# try:
#     N_EPISODES = 5
#     for ep in range(N_EPISODES):
#         print(f"\n===== ÉPISODE {ep+1}/{N_EPISODES} =====")
#         policy.reset_episode()

#         if episode_loop.plotter is not None:
#             episode_loop.plotter.reset()
#         if episode_loop.loss_plotter is not None:
#             episode_loop.loss_plotter.reset()

#         episode_loop.run_episode()

#         #Sauvegarde après chaque épisode (checkpoint)
#         os.makedirs("RLSTD_params", exist_ok=True)
#         policy.save_params(f"RLSTD_params/rlstd_phase1_ep{ep+1}.npz")

# except KeyboardInterrupt:
#     print("Interruption manuelle (Ctrl+C) détectée.")

# finally:
#     # Sauvegarde des paramètres RLSTD
#     policy.save_params("RLSTD_params/rlstd_phase1_last.npz")


#     # Arrêt et fermeture propre de Simulink / MATLAB
#     instance.eng.set_param(instance.sim_name, 'SimulationCommand', 'stop', nargout=0)
#     instance.close()






#############
##### Phase 2

# policy.train_value_function = True 
# policy.adapt_gains = True          
# policy.actions_set = 1
# policy.load_params("RLSTD_params/rlstd_phase1_last.npz")              

# episode_loop = ps.EpisodeLoop(
#     env=instance,
#     consigne=None,
#     policy=policy,
#     max_steps=3000,
#     policy_dt=0.1,
#     max_sim_time=30.0
# )

# # Planning d'epsilon (simple décroissance linéaire)
# N_EPISODES = 10
# eps_start = 1.0
# eps_end = 0.05

# try:
#     for ep in range(N_EPISODES):
#         print(f"\n===== PHASE 2 — ÉPISODE {ep+1}/{N_EPISODES} =====")

#         # Epsilon décroissant
#         alpha = ep / max(1, N_EPISODES-1)
#         policy.epsilon = eps_start + (eps_end - eps_start) * alpha
#         print(f"epsilon = {policy.epsilon:.3f}")

#         # Reset des états internes de la policy (PAS theta/A)
#         policy.reset_episode()

#         # Optionnel mais souvent utile : repartir des gains P2 à chaque épisode
#         policy.Kp = 0.4
#         policy.Ki = 0.04
#         policy.Kd = 0.0

#         # Reset des plots
#         if episode_loop.plotter is not None:
#             episode_loop.plotter.reset()
#         if episode_loop.loss_plotter is not None:
#             episode_loop.loss_plotter.reset()

#         # Lancer un épisode
#         episode_loop.run_episode()

#         # Sauvegarde après chaque épisode (checkpoint)
#         os.makedirs("RLSTD_params", exist_ok=True)
#         policy.save_params(f"RLSTD_params/rlstd_phase2_ep{ep+1}.npz")

# except KeyboardInterrupt:
#     print("Interruption manuelle (Ctrl+C).")

# finally:
#     # Sauvegarde finale
#     policy.save_params("RLSTD_params/rlstd_phase2_last.npz")

#     instance.eng.set_param(instance.sim_name, 'SimulationCommand', 'stop', nargout=0)
#     instance.close()




#############
##### Phase 3


# 🔹 Charger les paramètres appris en phase 2
policy.load_params("RLSTD_params/rlstd_phase2_last.npz")

# --- Configuration Phase 3 (test) ---
policy.train_value_function = False   # plus de mise à jour RLSTD
policy.adapt_gains = True            # on applique la politique RL greedy
policy.actions_set = 1               # set riche de 25 actions
policy.epsilon = 0.0                 # aucune exploration

episode_loop = ps.EpisodeLoop(
    env=instance,
    consigne=None,          # ou une consigne fixée pour le test
    policy=policy,
    max_steps=3000,
    policy_dt=0.1,
    max_sim_time=30.0
)

try:
    N_EPISODES = 5  # par ex : 5 seeds différentes pour évaluer la robustesse
    for ep in range(N_EPISODES):
        print(f"\n===== PHASE 3 — ÉPISODE {ep+1}/{N_EPISODES} =====")

        # Reset des états internes de la policy (pas de reset de theta/A)
        policy.reset_episode()

        # On peut repartir des gains initiaux P2 à chaque épisode
        policy.Kp = 0.4
        policy.Ki = 0.04
        policy.Kd = 0.0

        # Reset des graphiques
        if episode_loop.plotter is not None:
            episode_loop.plotter.reset()
        if episode_loop.loss_plotter is not None:
            episode_loop.loss_plotter.reset()

        # Lancer l'épisode (sans apprentissage, greedy pur)
        episode_loop.run_episode()

        # Optionnel : sauvegarder les logs/performances par épisode
        # (theta/A ne changent plus, donc pas la peine de les sauver à chaque fois)

except KeyboardInterrupt:
    print("Interruption manuelle (Ctrl+C).")

finally:
    # Sauvegarde des paramètres (facultatif : ils n'ont pas changé en phase 3)
    os.makedirs("RLSTD_params", exist_ok=True)
    policy.save_params("RLSTD_params/rlstd_phase3_last.npz")

    instance.eng.set_param(instance.sim_name, 'SimulationCommand', 'stop', nargout=0)
    instance.close()