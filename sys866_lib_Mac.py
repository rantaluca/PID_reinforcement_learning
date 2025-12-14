import numpy as np
import math
import time
import matlab.engine
import os

class SimulinkInstance:
    """
    Classe pour gérer une instance Simulink.
    """
    def __init__(self, sim_name, pid_block, consigne_block, consigne_block, init_Kp, init_Ki, init_Kd, dt_sim=0.01, pre_script=None, pre_script=None):
        self.sim_name = sim_name
        self.pid_block = pid_block
        self.consigne_block = consigne_block
        if pre_script is not None:  
            self.pre_script = pre_script
        self.consigne_block = consigne_block
        if pre_script is not None:  
            self.pre_script = pre_script
        self.dt_sim = dt_sim
        self.current_time = 0.0

        # Connection au moteur MATLAB
        self.eng = matlab.engine.start_matlab()
        print(f"MATLAB Engine démarré. Chargement du modèle {self.sim_name}.slx...")

        if pre_script is not None:
            self.eng.run(self.pre_script, nargout=0)
            print(f"Script préliminaire {self.pre_script} exécuté.")

        if pre_script is not None:
            self.eng.run(self.pre_script, nargout=0)
            print(f"Script préliminaire {self.pre_script} exécuté.")

        self.eng.load_system(self.sim_name)
        print(f"Modèle {self.sim_name}.slx chargé.")

        #self.set_pid_params(Kp=init_Kp, Ki=init_Ki, Kd=init_Kd)

        # Configuration de la simulatio

        self.eng.set_param(self.sim_name, 'BlockReduction', 'off', nargout=0)
        self.eng.set_param(self.sim_name, 'SimulationMode', 'Normal', nargout=0)  # Mode normal
        self.eng.set_param(self.sim_name, 'StopTime', 'inf', nargout=0) # Simulation infinie


        # self.eng.set_param(self.sim_name, 'SolverType', 'Fixed-step', nargout=0)
        # self.eng.set_param(self.sim_name, 'Solver', 'ode4', nargout=0)
        # self.eng.set_param(self.sim_name, 'FixedStep', str(self.dt_sim), nargout=0)
        # self.eng.set_param(self.sim_name, 'SolverType', 'Fixed-step', nargout=0)
        # self.eng.set_param(self.sim_name, 'Solver', 'ode4', nargout=0)
        # self.eng.set_param(self.sim_name, 'FixedStep', str(self.dt_sim), nargout=0)

    def set_pid_params(self, Kp, Ki, Kd):
        """
        Met à jour les paramètres du PID dans le modèle Simulink.
        """
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'P', str(Kp), nargout=0)
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'I', str(Ki), nargout=0)
        self.eng.set_param(f"{self.sim_name}/{self.pid_block}", 'D', str(Kd), nargout=0)
        #print(f"Paramètres PID mis à jour: Kp={Kp}, Ki={Ki}, Kd={Kd}")

    def get_pid_params(self):
        """
        Récupère les paramètres actuels du PID depuis le modèle Simulink.
        """
        Kp = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'P'))
        Ki = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'I'))
        Kd = float(self.eng.get_param(f"{self.sim_name}/{self.pid_block}", 'D'))
        return Kp, Ki, Kd
    
    def set_consigne(self, consigne):
        self.eng.set_param(f"{self.sim_name}/{self.consigne_block}", 'After', str(consigne), nargout=0)
        # or 'FinalValue' depending on your Simulink version
        print(f"Consigne mise à jour (After / FinalValue): {consigne}")
    
    def set_consigne(self, consigne):
        self.eng.set_param(f"{self.sim_name}/{self.consigne_block}", 'After', str(consigne), nargout=0)
        # or 'FinalValue' depending on your Simulink version
        print(f"Consigne mise à jour (After / FinalValue): {consigne}")
    
    # get observation # (récupère les observations courantes de la simulation) 
    def get_observation(self):
        """
        Lit la derniere ligne dans logs.csv
        """
        # use os to read the last line of logs.csv, separated by commas
        if not os.path.exists('logs.csv'):
            return None
        if not os.path.exists('logs.csv'):
            return None
        with open('logs.csv', 'r') as f:
            lines = f.readlines()
            if not lines:
                return None
            if not lines:
                return None
            last_line = lines[-1]
            values = last_line.strip().split(',')   
        consigne = float(values[0])
        error = float(values[1])
        command = float(values[2])
        output = float(values[3])
        observation = (consigne, error, command, output)
        return observation
        
    def step(self):
        last_t = self.get_sim_time()
        while True:
            t = self.get_sim_time()
            if t > last_t:
                break
        return self.get_observation(), {}

    
    def get_sim_time(self):
        """
        Récupère le temps de simulation actuel.
        """
        sim_time = float(self.eng.get_param(self.sim_name, 'SimulationTime'))
        return sim_time
    
    def reset(self):
        """
        Réinitialise la simulation.
        """
        self.eng.set_param(self.sim_name, 'SimulationCommand', 'stop', nargout=0)
        self.eng.set_param(self.sim_name, 'StartTime', '0', nargout=0)
        self.eng.set_param(self.sim_name, 'StopTime', 'inf', nargout=0)
        print("Simulation réinitialisée et démarrée.")
    
    def reset(self):
        """
        Réinitialise la simulation.
        """
        self.eng.set_param(self.sim_name, 'SimulationCommand', 'stop', nargout=0)
        self.eng.set_param(self.sim_name, 'StartTime', '0', nargout=0)
        self.eng.set_param(self.sim_name, 'StopTime', 'inf', nargout=0)
        print("Simulation réinitialisée et démarrée.")

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

        # Calcul de l'erreur statique

        error = observation[1]
        sortie = observation[3]
        if self.consigne is None:
            self.consigne = observation[0]

        if self.consigne == 0:
            error_norm = 0.0
        else:        if self.consigne == 0:
            error_norm = 0.0
        else:
                error_norm = error / self.consigne

        # Calcul de l'overshoot

        overshoot = max(0.0, sortie - self.consigne)
        if self.consigne == 0:  
            overshoot_norm = 0.0
        else:
            if self.consigne == 0:  
            overshoot_norm = 0.0
        else:
            overshoot_norm = overshoot / self.consigne

        # Calcul de l'indicateur de non-crunning_lossonvergence

        if (abs(error) >= self.consigne * (1.0 - self.consigne_tresh)):
            conv_norm = 1.0
        else:
            conv_norm = 0.0

        # Calcul de la fonction de coût 
        self.current_loss = (
            self.loss_params['error'] * error_norm**2 +
            self.loss_params['dep'] * overshoot_norm**2 +
            self.loss_params['conv'] * conv_norm)
        
        self.running_loss += self.current_loss

        return self.current_loss, self.running_loss
        

    def next_action(self, observation):
        return None
        # return Kp, Ki, Kd

import matplotlib.pyplot as plt
from collections import deque

import matplotlib.pyplot as plt

class OfflinePlotter:
    """
    Plot non temps réel.
    Stocke les données et sauvegarde à la fin.
    """
    def __init__(self):
        self.times = []
        self.consigne = []
        self.error = []
        self.command = []
        self.output = []

    def update(self, t, obs):
        self.times.append(t)
        self.consigne.append(obs[0])
        self.error.append(obs[1])
        self.command.append(obs[2])
        self.output.append(obs[3])

    def save(self, filename):
        fig, ax = plt.subplots()

        ax.plot(self.times, self.consigne, label="Consigne")
        ax.plot(self.times, self.error, label="Erreur")
        ax.plot(self.times, self.command, label="Commande")
        ax.plot(self.times, self.output, label="Sortie")

        ax.set_title("Évolution du système")
        ax.set_xlabel("Temps (s)")
        ax.set_ylabel("Valeurs")
        ax.grid(True)
        ax.legend()

        fig.savefig(filename)
        plt.close(fig)


class OfflineLossPlotter:
    """
    Plot offline de la loss
    """
    def __init__(self):
        self.times = []
        self.current_loss = []
        self.running_loss = []

    def update(self, t, current_loss, running_loss):
        self.times.append(t)
        self.current_loss.append(current_loss)
        self.running_loss.append(running_loss)

    def save(self, filename):
        fig, ax = plt.subplots()

        ax.plot(self.times, self.current_loss, label="Loss courante")
        ax.plot(self.times, self.running_loss, label="Loss cumulée")

        ax.set_title("Évolution de la loss")
        ax.set_xlabel("Temps (s)")
        ax.set_ylabel("Loss")
        ax.grid(True)
        ax.legend()

        fig.savefig(filename)
        plt.close(fig)


class RealTimePlotter:
    """
    Gère une figure matplotlib en temps réel 
    """
    def __init__(self, max_points=500):
        plt.ion()  # mode interactif

        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.consigne_hist = deque(maxlen=max_points)
        self.error_hist = deque(maxlen=max_points)
        #self.command_hist = deque(maxlen=max_points)
        self.output_hist = deque(maxlen=max_points)

        self.fig, self.ax = plt.subplots()
        self.line_consigne, = self.ax.plot([], [], label="Consigne")
        self.line_error, = self.ax.plot([], [], label="Erreur")
        #self.line_command, = self.ax.plot([], [], label="Commande")
        self.line_output, = self.ax.plot([], [], label="Sortie")

        self.ax.legend()
        self.ax.set_title("Évolution de la sortie du système en Temps Réel")
        self.ax.set_xlabel("Temps (s)")
        self.ax.set_ylabel("Valeurs")
        self.ax.grid(True)

    def update(self, t, obs):
        """
        Met à jour les courbes en temps réel.
        """
        self.times.append(t)
        self.consigne_hist.append(obs[0])
        self.error_hist.append(obs[1])
        #self.command_hist.append(obs[2])
        self.output_hist.append(obs[3])

        self.line_consigne.set_data(self.times, self.consigne_hist)
        self.line_error.set_data(self.times, self.error_hist)
        #self.line_command.set_data(self.times, self.command_hist)
        self.line_output.set_data(self.times, self.output_hist)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save(self, filename):
        fig, ax = plt.subplots()

        ax.plot(self.times, self.consigne_hist, label="Consigne")
        ax.plot(self.times, self.error_hist, label="Erreur")
        #ax.plot(self.times, self.command_hist, label="Commande")
        ax.plot(self.times, self.output_hist, label="Sortie")

        ax.set_title("Évolution du système")
        ax.set_xlabel("Temps (s)")
        ax.set_ylabel("Valeurs")
        ax.grid(True)
        ax.legend()

        fig.savefig(filename)
        plt.close(fig)

    def close(self):
        plt.ioff()
        plt.close(self.fig)

class LossPlotter:
    """
    Gère une figure matplotlib pour la loss en temps réel 
    """
    def __init__(self, max_points=500):
        plt.ion()  # mode interactif

        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.current_loss_hist = deque(maxlen=max_points)
        self.running_loss_hist = deque(maxlen=max_points)

        self.fig, self.ax = plt.subplots()
        self.line_current_loss, = self.ax.plot([], [], label="Perte Courante")
        self.line_running_loss, = self.ax.plot([], [], label="Perte Totale")

        self.ax.legend()
        self.ax.set_title("Évolution de la Loss en Temps Réel")
        self.ax.set_xlabel("Temps (s)")
        self.ax.set_ylabel("Loss")
        self.ax.grid(True)

    def update(self, t, current_loss, running_loss = None):
        """
        Met à jour les courbes de loss en temps réel.
        """
        self.times.append(t)
        self.current_loss_hist.append(current_loss)
        self.running_loss_hist.append(running_loss)

        self.line_current_loss.set_data(self.times, self.current_loss_hist)
        if running_loss is not None:
            self.line_running_loss.set_data(self.times, self.running_loss_hist)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save(self, filename):
        fig, ax = plt.subplots()

        ax.plot(self.times, self.current_loss_hist, label="Perte Courante")
        ax.plot(self.times, self.running_loss_hist, label="Perte Totale")

        ax.set_title("Évolution de la Loss")
        ax.set_xlabel("Temps (s)")
        ax.set_ylabel("Loss")
        ax.grid(True)
        ax.legend()

        fig.savefig(filename)
        plt.close(fig)

    def close(self):
        plt.ioff()
        plt.close(self.fig)

class PIgainsPlotter():
    """
    Plot les gains Kp et Ki en temps réel
    """

    def __init__(self, max_points=500):
        plt.ion()  # mode interactif

        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.Kp_hist = deque(maxlen=max_points)
        self.Ki_hist = deque(maxlen=max_points)

        self.fig, self.ax = plt.subplots()
        self.line_Kp, = self.ax.plot([], [], label="Kp")
        self.line_Ki, = self.ax.plot([], [], label="Ki")

        self.ax.legend()
        self.ax.set_title("Évolution des gains Kp et Ki en Temps Réel")
        self.ax.set_xlabel("Temps (s)")
        self.ax.set_ylabel("Gains")
        self.ax.grid(True)

    def update(self, t, Kp, Ki):
        """
        Met à jour les courbes des gains en temps réel.
        """
        self.times.append(t)
        self.Kp_hist.append(Kp)
        self.Ki_hist.append(Ki)

        self.line_Kp.set_data(self.times, self.Kp_hist)
        self.line_Ki.set_data(self.times, self.Ki_hist)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save(self, filename):
        fig, ax = plt.subplots()

        ax.plot(self.times, self.Kp_hist, label="Kp")
        ax.plot(self.times, self.Ki_hist, label="Ki")

        ax.set_title("Évolution des gains Kp et Ki")
        ax.set_xlabel("Temps (s)")
        ax.set_ylabel("Gains")
        ax.grid(True)
        ax.legend()

        fig.savefig(filename)
        plt.close(fig)
    
    def close(self):
        plt.ioff()
        plt.close(self.fig)

class EpisodeLoop:
    """
    Classe pour gérer une boucle d'épisode.
    """
    def __init__(self, env, consigne, policy, max_steps=1000, max_time=30, policy_dt=0.1, plot=True):
        self.env = env
        self.consigne = consigne  
        self.policy = policy
        self.max_steps = max_steps
        self.max_time = max_time
        self.policy_dt = policy_dt
        if plot:
            self.plotter = RealTimePlotter()
            self.loss_plotter = LossPlotter()
            self.gains_plotter = PIgainsPlotter()
        else:
            self.plotter = None
            self.loss_plotter = None

    def run_episode(self):
        """
        Exécute une boucle d'épisode.
        """
        # Démarre la simulation
        #self.env.start_sim(total_time=self.sim_total_time)

        # sauavegarde l'ancien logs.csv dans experience du date et heure
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        policy_name = self.policy.__class__.__name__
        # créer le dossier experiences 
        os.makedirs("experiences", exist_ok=True)

        # archiver l'ancien csv
        if os.path.isfile('logs.csv'):
            os.rename('logs.csv', f'experiences/logs_{timestamp}_{policy_name}.csv')
            
        if self.consigne is not None:
            self.env.set_consigne(self.consigne)

        # On s'assure que la simu est arrêtée
        self.env.eng.set_param(self.env.sim_name, 'SimulationCommand', 'stop', nargout=0)

        # On remet le temps de départ
        self.env.eng.set_param(self.env.sim_name, 'StartTime', '0', nargout=0)
        self.env.eng.set_param(self.env.sim_name, 'StopTime', 'inf', nargout=0)

        # On lance la simulation en real time
        self.env.eng.set_param(self.env.sim_name, 'SimulationCommand', 'start', nargout=0)
        print("Simulation Simulink démarrée.")

        print("Simulation Simulink démarrée.")

        for step in range(self.max_steps):
            # Récupérer l'observation actuelle et le temps de simulation
            observation, info = self.env.step()
            if observation is None:
                print("Aucune observation reçue, premier pas de simulation en cours...")
                continue
            
            if observation[0] == 0:
                print("Consigne nulle détectée")
                continue

            if observation is None:
                print("Aucune observation reçue, premier pas de simulation en cours...")
                continue
            
            if observation[0] == 0:
                print("Consigne nulle détectée")
                continue

            sim_time = self.env.get_sim_time()

            if sim_time >= self.max_time:
                print(f"Temps maximum de l'épisode atteint: {sim_time:.2f}s")
                break

            # Calcul de la perte
            current_loss, running_loss = self.policy.compute_loss(observation)
            if self.loss_plotter is not None:
                self.loss_plotter.update(sim_time, current_loss, None)
                #print(f"Temps: {sim_time:.2f}s, Observation: {observation}, Perte courante: {current_loss:.4f}, Perte totale: {running_loss:.4f}")
            #else:
                #print(f"Temps: {sim_time:.2f}s, Observation: {observation}, Perte courante: {current_loss:.4f}, Perte totale: {running_loss:.4f}")

            if self.plotter is not None:
                self.plotter.update(sim_time, observation)

            # Obtenir la prochaine action tout les policy_dt
            if step % int(self.policy_dt / self.env.dt_sim) == 0:
                action = self.policy.next_action(observation)
                if action is not None:
                    Kp, Ki, Kd = action
                    self.env.set_pid_params(Kp, Ki, Kd)
                    if self.gains_plotter is not None:
                        self.gains_plotter.update(sim_time, Kp, Ki)

        print("Fin épisode.")
        mean_running_loss = self.policy.running_loss / (step + 1)
        print(f"Perte totale de l'épisode: {self.policy.running_loss:.4f}")
        print("loss moyenne par step: ", mean_running_loss)
        
        # Sauvegarde des plots          
        print("Sauvegarde des plots.")
        if self.plotter is not None:
            self.plotter.save(f"plots/eval_plot_{timestamp}_{policy_name}.png")
            self.plotter.close()
        if self.loss_plotter is not None:
            self.loss_plotter.save(f"plots/eval_loss_plot_{timestamp}_{policy_name}.png")
            self.loss_plotter.close()
        if self.gains_plotter is not None:
            self.gains_plotter.save(f"plots/eval_gains_plot_{timestamp}_{policy_name}.png")
            self.gains_plotter.close()
            
        return self.policy.running_loss, mean_running_loss