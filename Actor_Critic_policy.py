import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import numpy as np
import sys866_lib as ps
# si windows utilisez la ligne suivante à la place de la précédente
# import sys866_lib_windows as ps

class ActorCriticPolicy(ps.Policy,nn.Module):
    def __init__(self, consigne=None, consigne_tresh=0.97, loss_params={'error':5.0, 'dep':0.5, 'conv':0.3}, Kp_init = 0.4, Ki_init=0.04, dt=0.1, actor_lr=1e-5, critic_lr=5e-5, gamma=0.90):
        ps.Policy.__init__(self, consigne, consigne_tresh, loss_params, dt)
        nn.Module.__init__(self)

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = torch.device(device)

        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(6 , 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 4), # 4 outputs moyennes et var des distributions Kp, Ki
        )

        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(6, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        # envoie des réseaux sur le device
        self.actor.to(self.device)
        self.critic.to(self.device)

        # Initialisation des variables pour le calcul des features
        self.prev_error = 0.0
        self.cumulative_error = 0.0
        self.Integral_ref = 1.0

        # PID internal state (mirrors Simulink)
        self.Kp = Kp_init
        self.Ki = Ki_init

        self.Kp_min, self.Kp_max = 0.0, 2.0
        self.Ki_min, self.Ki_max = 0.0, 0.5


        # Normalization references
        self.Kp_ref = 1.0
        self.Ki_ref = 0.1


        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=critic_lr)
        self.gamma = gamma

        # Memory
        self.last_features = None
        self.last_action_tensor = None
        self.last_reward = 0.0

    def load_actor_critic(self, actor_path, critic_path):
        """
        Load the actor and critic networks from saved files.
        
        """
        self.actor.load_state_dict(torch.load(actor_path))
        self.critic.load_state_dict(torch.load(critic_path))
        print("Actor-Critic models loaded.")

    def save_actor_critic(self, actor_path, critic_path):
        """
        Save the actor and critic networks to files.
        
        """
        torch.save(self.actor.state_dict(), actor_path)
        torch.save(self.critic.state_dict(), critic_path)


    def compute_loss(self, observation):

        ### Fonction de coût via classe parente

        current_loss, running_loss = super().compute_loss(observation)
        reward = -np.tanh(current_loss / 50.0)
        self.last_reward = reward
        return current_loss, running_loss
    

    def compute_features(self, observation, update_state=False):
        consigne, error, command, output = observation

        consigne = max(abs(consigne), 1e-3)

        if update_state:
            self.cumulative_error += error * self.dt
            derivative_error = (error - self.prev_error) / self.dt
            self.prev_error = error
        else:
            derivative_error = (error - self.prev_error) / self.dt

        I_max = 500.0

        phi = np.array([
            1.0,
            np.tanh(error / consigne),
            np.tanh(derivative_error / consigne),
            np.tanh(self.cumulative_error / I_max),
            self.Kp / self.Kp_max,
            self.Ki / self.Ki_max,
        ], dtype=np.float32)

        return phi
    
    def get_feature_size(self):
        return len(self.compute_features(np.array([0.0,0.0, 0.0, 0.0])))
        
    def update_policy(self, observation, next_observation, done):
        if self.last_features is None:
            return

        s = self.last_features
        s2 = torch.tensor(
            self.compute_features(next_observation, update_state=False),
            dtype=torch.float32,
            device=self.device
        )

        # Actor
        params = self.actor(s)
        mu = torch.tanh(params[:2])
        log_std = torch.clamp(params[2:], -3, 0)
        std = torch.exp(log_std)
        dist = Normal(mu, std)

        log_prob = dist.log_prob(self.last_action_tensor).sum()
        entropy = dist.entropy().sum()

        # Critic
        value = self.critic(s).squeeze()
        with torch.no_grad():
            next_value = self.critic(s2).squeeze()
            target = self.last_reward + (0.0 if done else self.gamma * next_value)

        advantage = target - value
        #normalize advantage
        advantage = advantage / target.abs().clamp(min=1.0)

        # Critic update
        critic_loss = advantage.pow(2)
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()

        # Actor update
        actor_loss = -log_prob * advantage.detach() - 0.005 * entropy
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()



    def next_action(self, observation):
        features = self.compute_features(observation, update_state=True)
        s = torch.tensor(features, dtype=torch.float32, device=self.device)
        self.last_features = s

        with torch.no_grad():
            params = self.actor(s)

            mu = torch.tanh(params[:2])          # [-1, 1]
            log_std = torch.clamp(params[2:], -3, 0)
            std = torch.exp(log_std)

            dist = Normal(mu, std)
            action = dist.sample()
            self.last_action_tensor = action

        #  Safe updates
        dKp = 0.05 * action[0].item()
        dKi = 0.01 * action[1].item()

        self.Kp = np.clip(self.Kp + dKp, self.Kp_min, self.Kp_max)
        self.Ki = np.clip(self.Ki + dKi, self.Ki_min, self.Ki_max)

        return self.Kp, self.Ki, 0.0

class EpisodeLoopActorCritic(ps.EpisodeLoop):
    """
    Boucle d'épisode spécialisée pour Actor-Critic
    (hérite de EpisodeLoop mais redéfinit run_episode)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.policy_steps = int(self.policy_dt / self.env.dt_sim)

    def run_episode(self):
        # Initialisation simulation 
        self.env.eng.set_param(self.env.sim_name, 'SimulationCommand', 'stop', nargout=0)
        self.env.eng.set_param(self.env.sim_name, 'StartTime', '0', nargout=0)
        self.env.eng.set_param(self.env.sim_name, 'StopTime', 'inf', nargout=0)
        self.env.eng.set_param(self.env.sim_name, 'SimulationCommand', 'start', nargout=0)

        print("Simulation Simulink démarrée (Actor-Critic).")

        obs = None
        last_policy_obs = None
        done = False

        for step in range(self.max_steps):

            # Step simulation 
            obs, _ = self.env.step()
            if obs is None or obs[0] == 0:
                continue

            sim_time = self.env.get_sim_time()
            if sim_time >= self.max_time:
                break

            # Reward (pour l'action précédente) 
            current_loss, running_loss =  self.policy.compute_loss(obs)
            

            # Décision politique toute les policy_steps
            if step % self.policy_steps == 0:
                # Update RL avec transition complète
                if last_policy_obs is not None:
                    print(f"Temps: {sim_time:.2f}s, Observation: {last_policy_obs}, Perte courante: {current_loss:.4f}, Perte totale: {running_loss:.4f}")
                    self.policy.update_policy(
                        observation=last_policy_obs,
                        next_observation=obs,
                        done=done
                    )

                # Nouvelle action
                action = self.policy.next_action(obs)
                if action is not None:
                    Kp, Ki, Kd = action
                    self.env.set_pid_params(Kp, Ki, Kd)

                last_policy_obs = obs

            if done:
                break

        print("Fin épisode Actor-Critic.")
