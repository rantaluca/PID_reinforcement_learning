import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import numpy as np
import sys866_lib as ps

class ActorCriticPolicy(ps.Policy,nn.Module):
    def __init__(self, consigne=None, consigne_tresh=0.97, loss_params={'error':5.0, 'dep':0.5, 'conv':0.3}, dt=0.1, actor_lr=1e-4, critic_lr=1e-3, gamma=0.99):
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
            nn.Linear(64, 6), # 6 outputs moyennes et var des distributions Kp, Ki, Kd
            nn.Softplus()  # Softmax avec valeurs positives pour Kp, Ki, Kd
        )

        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(8, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

        # Initialisation des variables pour le calcul des features
        self.prev_error = 0.0
        self.cumulative_error = 0.0
        self.Integral_ref = 1.0

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=critic_lr)
        self.gamma = gamma

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

        error_square = error ** 2

        ## Calcul des features pour la VFA (phi)

        #1 Le biais vaut 1

        bias = 1.0

        #2 Calcul de l'erreur statique normalisée

        if consigne == 0:
            error_norm = 0.0
        else:
            error_norm = error / consigne

        #3 Calcul de l'erreur carrée

        error_norm_squared = error_square / consigne

        #4 Calcul de l'erreur dérivée

        derivative_error = error - self.prev_error
        if consigne == 0:
            derivative_error_norm = 0.0
        else:
            derivative_error_norm = derivative_error / consigne

        #5 Calcul de l'indicateur de non-convergence

        if (abs(error) >= consigne * (1.0 - self.consigne_tresh)):
            conv_norm = 1.0
        else:
            conv_norm = 0.0
        
        #6 Calcul de l'erreur intégrale nomalisée

        integrative_error_norm = max(-1.0, min(1.0, self.cumulative_error / self.Integral_ref))

        #78 Gains normalisés

        Kp_norm = self.Kp / self.Kp_ref
        Ki_norm = self.Ki / self.Ki_ref

        phi = np.array([
            bias,
            error_norm,
            error_norm_squared,
            derivative_error_norm,
            conv_norm,
            integrative_error_norm,
            Kp_norm,
            Ki_norm,
        ], dtype=float)

        self.prev_error = error

        return phi
    
    
    def get_feature_size(self):
        return len(self.compute_features(np.array([0.0,0.0, 0.0, 0.0])))
    
    def update_policy(self, observation, done):
        #tensor avec features   
        state = torch.FloatTensor(observation).to(self.device)

        # pred action
        action_params = self.actor(state)
        mu = action_params[:3]
        sigma = action_params[3:] + 1e-5  # Avoid zero std

        dist = Normal(mu, sigma)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum()

        # pred critic value
        value = self.critic(state)

        # reward
        reward = self.last_reward

        # pred du next state par critic
        next_state = torch.FloatTensor(observation).to(self.device)
        next_value = self.critic(next_state)

        # Calcul de la cible et de l'avantage
        target = reward + (1 - done) * self.gamma * next_value
        advantage = target - value

        # Mise à jour du critique
        critic_loss = advantage.pow(2).mean()
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Mise à jour de l'acteur
        actor_loss = -log_prob * advantage.detach()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()


    def next_action(self, observation):
        features = self.compute_features(observation)
        state = torch.FloatTensor(features).to(self.device)
        action_params = self.actor(state)
        mu = action_params[:3]
        return mu.detach().cpu().numpy()
