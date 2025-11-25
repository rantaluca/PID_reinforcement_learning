import numpy as np
import math
import time
class PID:
    """
    Classe du PID continu simple.
    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def set_parameters(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def control(self, error, dt):
        # intégrale de l'erreur
        self.integral += error * dt
        
        # approximation de la dérivée
        derivative = (error - self.prev_error) / dt

        # Sortie du PID
        u = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )
        self.prev_error = error
        return u
    
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


class EpisodeLoopPendulum(EpisodeLoop):
    """
    Classe pour gérer une boucle d'épisode spécifique au Pendulum.
    """
    def __init__(self, env, consigne, policy, pid, max_steps=1000):
        super().__init__(env, consigne, policy, pid, max_steps)  
        
    
    def run(self):
        obs, info = self.env.reset()
        self.pid.reset()
        dt = self.policy.dt

        for step in range(self.max_steps):
            start_time = time.time()


            # propre à Pendulum-v1 
            cos_t, sin_t, theta_dot = obs
            theta = math.atan2(sin_t, cos_t)

            # erreur de position
            error = self.policy.consigne - theta

            #Calcul de l'action via le PID
            u = self.pid.control(error, dt)

            # On applique la sortie du PID à l'environnement
            obs, reward, terminated, truncated, info = self.env.step(np.array([u], dtype=np.float32))

            # Calcul de la loss (pour logging/entrainement)
            new_parameters = self.policy.next_action(theta)
            if new_parameters is not None:
                #Mise à jour des paramètres du PID
                self.pid.set_parameters(*new_parameters)
            print(f"Step: {step}, Angle: {theta:.4f}, Action: {u:.4f}, Error: {error:.4f}")

            time.sleep(max(0.0, dt - (time.time() - start_time))) #sleep to maintain dt
