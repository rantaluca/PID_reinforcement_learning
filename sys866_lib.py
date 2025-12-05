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
        self.current_time = 0.0

        # Connection au moteur MATLAB
        self.eng = matlab.engine.start_matlab()
        print(f"MATLAB Engine démarré. Chargement du modèle {self.sim_name}.slx...")

        self.eng.load_system(self.sim_name)
        print(f"Modèle {self.sim_name}.slx chargé.")

        self.set_pid_params(Kp=init_Kp, Ki=init_Ki, Kd=init_Kd)

        # Configuration de la simulatio

        self.eng.set_param(self.sim_name, 'BlockReduction', 'off', nargout=0)

        self.eng.set_param(self.sim_name, 'SimulationMode', 'Normal', nargout=0)  # Mode normal
        self.eng.set_param(self.sim_name, 'StopTime', 'inf', nargout=0) # Simulation infinie
        self.eng.set_param(self.sim_name, 'FixedStep', str(self.dt_sim), nargout=0) # Pas fixe



        self.eng.set_param(self.sim_name, 'SimulationCommand', 'start', nargout=0)

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
    
    # get observation # (récupère les observations courantes de la simulation) 
    def get_observation(self):
        """
        Lit le vecteur [Consigne, Error, Command, Output] directement
        depuis l'entrée du bloc 'Sensors'.
        """

        cmd = f"get_param('{self.sim_name}/Sensors', 'RuntimeObject').OutputPort(1).Data"

        try:
            res = self.eng.eval(cmd)
            data = np.array(res).flatten()
            return data

        except Exception as e:
            print(f"Erreur lecture capteurs (attente init...): {e}")
            return np.array([0.0, 0.0, 0.0, 0.0])

        
    def step(self):
        """Step la simulation d'un pas de temps"""
        self.eng.set_param(self.sim_name, 'SimulationCommand', 'step', nargout=0)
        obs = self.get_observation()
        return obs, {}
    
    def get_sim_time(self):
        """
        Récupère le temps de simulation actuel.
        """
        sim_time = float(self.eng.get_param(self.sim_name, 'SimulationTime'))
        return sim_time

    def close(self):
        """
        Ferme l'instance Simulink.
        """
        self.eng.set_param(self.sim_name, 'SimulationCommand', 'stop', nargout=0)
        self.eng.quit()
        print("MATLAB Engine fermé.")   

class Policy:
    """
    Template de politique
    """

    def __init__(self, consigne=None, consigne_tresh=0.97, loss_params={'error':1.0, 'dep':0.5, 'conv':0.3}, dt=0.1):
        self.convergence_time = 0.0 
        self.running_loss = 0.0
        self.current_loss = 0.0
        self.loss_params = loss_params
        self.consigne = consigne
        self.consigne_tresh = consigne_tresh
        self.dt = dt
    
    def compute_loss(self, observation):
        print("Observation reçue pour calcul de la loss:", observation)
        # calcul de l'erreur
        error = observation[1]
        sortie = observation[3]
        if self.consigne is None:
            self.consigne = observation[0]
        
        # calcul de l'overshoot
        overshoot = max(0.0, sortie - self.consigne)
        
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
    def __init__(self, env, consigne, policy, max_steps=1000, policy_dt=0.1):
        self.env = env
        self.consigne = consigne  
        self.policy = policy
        self.max_steps = max_steps
        self.policy_dt = policy_dt

    def run_episode(self):
        """
        Exécute une boucle d'épisode.
        """
        # Démarre la simulation
        #self.env.start_sim(total_time=self.sim_total_time)
        for step in range(self.max_steps):
            # Récupérer l'observation actuelle et le temps de simulation
            observation, info = self.env.step()
            sim_time = self.env.get_sim_time()

            # Calcul de la perte
            current_loss, running_loss = self.policy.compute_loss(observation)
            print(f"Temps: {sim_time:.2f}s, Observation: {observation}, Perte courante: {current_loss:.4f}, Perte totale: {running_loss:.4f}")

            # Obtenir la prochaine action tout les policy_dt
            if step % int(self.policy_dt / self.env.dt_sim) == 0:
                action = self.policy.next_action(observation)
                if action is not None:
                    Kp, Ki, Kd = action
                    self.env.set_pid_params(Kp, Ki, Kd)