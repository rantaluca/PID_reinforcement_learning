import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import numpy as np
import sys866_lib as ps

class ActorCriticPolicy(ps.Policy,nn.Module):
    def __init__(self, consigne=None, consigne_tresh=0.97, loss_params={'error':5.0, 'dep':0.5, 'conv':0.3}, Kp_init = 0.4, Ki_init=0.04, dt=0.1, actor_lr=1e-4, critic_lr=1e-3, gamma=0.99):
        ps.Policy.__init__(self, consigne, consigne_tresh, loss_params, dt)
        nn.Module.__init__(self)

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = torch.device(device)

        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(8 , 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 4), # 4 outputs moyennes et var des distributions Kp, Ki
            nn.Softplus()  # Softmax avec valeurs positives pour Kp, Ki
        )

        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(8, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
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

        # Normalization references
        self.Kp_ref = 1.0
        self.Ki_ref = 0.1


        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=critic_lr)
        self.gamma = gamma

        self.last_reward = 0.0

    def load_actor_critic(self, actor_path, critic_path):
        """
        Load the actor and critic networks from saved files.
        
        """
        self.actor.load_state_dict(torch.load(actor_path))
        self.critic.load_state_dict(torch.load(critic_path))


    def compute_loss(self, observation):

        ### Fonction de coût via classe parente

        current_loss, running_loss = super().compute_loss(observation)
        self.last_reward = -current_loss
        return current_loss, running_loss
    

    def compute_features(self, observation):
        consigne, error, command, output = observation
        consigne = float(consigne)
        error = float(error)

        # update integral (PI feature)
        self.cumulative_error += error * self.dt

        bias = 1.0

        if abs(consigne) < 1e-8:
            error_norm = 0.0
            error_norm_squared = 0.0
            derivative_error_norm = 0.0
            conv_norm = 0.0
        else:
            error_norm = error / consigne
            error_norm_squared = (error * error) / (consigne * consigne)

            derivative_error = (error - self.prev_error) / self.dt
            derivative_error_norm = derivative_error / consigne

            conv_norm = 1.0 if abs(error) >= abs(consigne) * (1.0 - self.consigne_tresh) else 0.0

        integrative_error_norm = max(-1.0, min(1.0, self.cumulative_error / self.Integral_ref))

        # these must exist (from ps.Policy) or you should guard them
        Kp_norm = float(self.Kp / self.Kp_ref)
        Ki_norm = float(self.Ki / self.Ki_ref)

        phi = np.array([
            bias,
            error_norm,
            error_norm_squared,
            derivative_error_norm,
            conv_norm,
            integrative_error_norm,
            Kp_norm,
            Ki_norm,
        ], dtype=np.float32)

        self.prev_error = error
        return phi
    
    
    def get_feature_size(self):
        return len(self.compute_features(np.array([0.0,0.0, 0.0, 0.0])))
    
    def update_policy(self, observation, next_observation, done):
        
        #tensor avec features   
        s = torch.tensor(self.compute_features(observation), dtype=torch.float32, device=self.device)
        s2 = torch.tensor(self.compute_features(next_observation), dtype=torch.float32, device=self.device)

        # pred action
        action_params = self.actor(s)
        mu = action_params[:2]
        sigma = action_params[2:] + 1e-5

        dist = Normal(mu, sigma)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum()

        # pred critic value
        value = self.critic(s).squeeze(-1)

        with torch.no_grad():
            next_value = self.critic(s2).squeeze(-1)
            target = torch.tensor(self.last_reward, device=self.device) + (0.0 if done else self.gamma * next_value)
            advantage = target - value

        # critic update
        critic_loss = advantage.pow(2)
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # actor update
        actor_loss = -(log_prob * advantage.detach())
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # return action if you need it
        return action.detach().cpu().numpy(), 0.0


    def next_action(self, observation):
        features = self.compute_features(observation)
        s = torch.tensor(features, dtype=torch.float32, device=self.device)
        with torch.no_grad():
            action_params = self.actor(s)
            mu = action_params[:2]
        Kp, Ki = mu.cpu().numpy()

        self.Kp = float(Kp)
        self.Ki = float(Ki)

        return self.Kp, self.Ki, 0.0