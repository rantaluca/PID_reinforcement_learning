import numpy as np
import math
import time
import matlab.engine
import numpy as np

class SimulinkInstance:
    """
    Classe pour gérer une instance Simulink.
    """
    SIGNAL_MAP = {
        'consigne': 1,  
        'error': 2,
        'command': 3,
        'output': 4
    }
    def __init__(self, sim_name, pid_block, init_Kp, init_Ki, init_Kd, dt_sim=0.01):
        self.sim_name = sim_name
        self.pid_block = pid_block
        self.dt_sim = dt_sim

        # Connection au moteur MATLAB
        self.eng = matlab.engine.start_matlab()
        print(f"MATLAB Engine démarré. Chargement du modèle {self.sim_name}.slx...")
        self.eng.load_system(self.model_name)
        print(f"Modèle {sim_name}.slx chargé.")
        self.set_pid_params(Kp=init_Kp, Ki=init_Ki, Kd=init_Kd)

        # Configuration de la simulation
        self.eng.set_param(self.sim_name, 'SimulationMode', 'Normal', nargout=0)  # Mode normal
        self.eng.set_param(self.sim_name, 'StopTime', 'inf', nargout=0) # Simulation infinie
        self.eng.set_param(self.sim_name, 'FixedStep', str(self.dt_sim), nargout=0) # Pas fixe

    def set_pid_params(self, Kp, Ki, Kd):
        """
        Met à jour les paramètres du PID dans le modèle Simulink.
        """
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'P', str(Kp), nargout=0)
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'I', str(Ki), nargout=0)
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'D', str(Kd), nargout=0)
        print(f"Paramètres PID mis à jour: Kp={Kp}, Ki={Ki}, Kd={Kd}")

    def get_pid_params(self):
        """
        Récupère les paramètres actuels du PID depuis le modèle Simulink.
        """
        Kp = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'P'))
        Ki = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'I'))
        Kd = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'D'))
        return Kp, Ki, Kd
    
    # step (# avance la simulation d'un pas de temps)

    # save_state (sauvegarde l'état courant en le retournant, pour qu'on puisse branch sur plusieurs simulations à partir d'un instant)

    # get observation # (récupère les observations courantes de la simulation)

    
    
    def close(self):
        """
        Ferme l'instance Simulink.
        """
        self.eng.quit()
        print("MATLAB Engine fermé.")   

class Policy:
    """
    Template de politique
    """

    def __init__(self, consigne, consigne_tresh=0.97, loss_params={'error':1.0, 'dep':0.5, 'conv':0.3}, dt=0.1):
        self.convergence_time = 0.0 
        self.running_loss = 0.0
        self.current_loss = 0.0
        self.loss_params = loss_params
        self.consigne = consigne
        self.consigne_tresh = consigne_tresh
        self.dt = dt
    
    def compute_loss(self, observation):
        # calcul de l'erreur
        error = self.consigne - observation[0]
        
        # calcul de l'overshoot
        overshoot = max(0.0, observation[0] - self.consigne)
        
        # verification si la consigne dans le treshhold
        if abs(error) < self.consigne * (1.0 - self.consigne_tresh):
            self.convergence_time += self.dt
        else:
            self.convergence_time = 0.0

        # calcul de la loss 
        self.current_loss = (
            self.loss_params['error'] * abs(error) +
            self.loss_params['dep'] * overshoot +
            self.loss_params['conv'] * self.convergence_time)
        
        self.running_loss += self.current_loss

        return self.current_loss, self.running_loss
        

    def next_action(self, observation):
        return None
        # return Kp, Ki, Kd


class EpisodeLoop:
    """
    Classe pour gérer une boucle d'épisode.
    """
    def __init__(self, env, consigne, policy, pid, max_steps=1000):
        self.env = env
        self.consigne = consigne  
        self.policy = policy
        self.max_steps = max_steps
        self.pid = pid

    # exemple de boucle d'épisode générique
    def run(self):
        observation, info = self.env.reset()
        self.pid.reset()
        dt = self.policy.dt

        for step in range(self.max_steps):

            # calcul entre deux steps
            start_time = time.time()

            
            #calcul de l'erreur
            error = self.consigne - observation[0]

            #Calcul de l'action via le PID
            action = self.pid.control(error,  dt)
            
            self.env.set_action(action)

            # nouvelle action via le PID
            observation, info = self.env.step()

            #Calcul des nouveaux paramètres du PID via la politique
            new_parameters = self.policy.next_action(observation,  dt)
            if new_parameters is not None:
                #Mise à jour des paramètres du PID
                self.pid.set_parameters(*new_parameters)
            
            time.sleep(max(0.0, dt - (time.time() - start_time)))