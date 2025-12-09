import sys866_lib as lib
import numpy as np

class RLSTDPolicy(lib.Policy):

    def __init__(
        self, 
        consigne, 
        consigne_tresh=0.97, 
        loss_params={'error':1.0, 'dep':0.5, 'conv':0.3}, 
        dt=0.1,
        Kp_ref = 5.0,
        Ki_ref = 1.0,
        dKp = 0.01,
        dKi = 0.001,
        dKp2 = 0.05,
        dKi2 = 0.005,
        beta = 100,
        epsilon = 0.1,
        gamma = 0.99,
        Integral_ref = 1, # Au plus ce facteur est élevé, au plus l'erreur intégrale normalisée pénalise
    ):

        super().__init__(
            consigne, 
            consigne_tresh, 
            loss_params, 
            dt)
    
        self.Kp_ref = Kp_ref
        self.Ki_ref = Ki_ref

        self.dKp = dKp
        self.dKi = dKi
        self.dKp2 = dKp2
        self.dKi2 = dKi2

        self.beta = beta
        self.epsilon = epsilon
        self.gamma = gamma

        self.prev_error = 0.0
        self.cumulative_error = 0.0
        self.Integral_ref = Integral_ref

        # Variables RLSTD
        self.number_of_features = 9
        self_theta = np.zeros(self.number_of_features)
        self.A = np.eye(self.number_of_features) * self.beta
        self.last_phi = None
        self.last_reward = 0.0




    
    def compute_features(self, observation):

        ## Calcul des features pour la VFA (phi)

        #1 Le biais vaut 1

        bias = 1.0

        #2 Calcul de l'erreur statique

        error = self.consigne - observation[0]
        error_norm = error / self.consigne

        #3 Calcul de l'erreur carrée

        error_norn_squared = error_norm^2

        #4 Calcul de l'erreur dérivée

        derivative_error = error - self.prev_error
        derivative_error_norm = derivative_error / self.consigne

        #5 Calcul de l'indicateur de non-convergence

        if (abs(error) >= self.consigne * (1.0 - self.consigne_tresh)):
            conv_norm = 1.0
        else:
            conv_norm = 0.0
        
        #6 Calcul de l'erreur intégrale nomalisée

        self.cumulative_error += error_norm * self.dt
        integrative_error_norm = lib.sat(self.cumulative_error / self.Integral_ref)

        #78 Gains normalisés

        Kp_norm = self.Kp / self.Kp_ref
        Ki_norm = self.Ki / self.Ki_ref

        phi = np.array([
            bias,
            error_norm,
            error_norn_squared,
            derivative_error_norm,
            conv_norm,
            integrative_error_norm,
            Kp_norm,
            Ki_norm,
        ], dtype=float)

        #On garde l'erreur actuelle pour le prochain step

        self.prev_error = error

        return phi
    




    def update_value_function(self, phi_next):
        
        ### Algorithme RLSTD


        ## Première itération

        if self.last_phi is None:

            # Première itération : on stocke juste pour la prochaine
            self.last_phi = phi_next.copy()
            return
        

        ## Changement des notations pour coller au rapport

        f = self.last_phi
        f_next = phi_next
        A = self.A
        theta = self.theta
        gamma = self.gamma
        r_t = self.last_reward


        # Calcul de l'erreur de Bellman
        v_t = float(theta @ f) #theta^T f dans rapport
        v_tplus1 = float(theta @ f_next) #theta^T f' dans rapport
        delta = r_t + gamma * v_tplus1 - v_t

        # Calcul de v (direction de correction)
        u = f - gamma * f_next
        v = A @ f  

        # Calcul de k (gain d'adaptation a * direction de correction v)
        a = 1.0 + float(u.T @ v)
        k = v / a

        # Mises à jour
        self.theta = theta + k * delta
        g_col = A @ u
        self.A = A - np.outer(k, g_col)
        self.last_phi = phi_next.copy()
        


    
    def compute_loss(self, observation):

        ### Fonction de coût via classe parente

        current_loss, running_loss = super().compute_loss(observation)

        self.last_reward = -current_loss

        return current_loss, running_loss, self.last_reward


    def enumerate_actions(self):
        
        ### Génération des 9 actions possibles (+/=/- dKp, +/=/- dKi)

        actions = []
        for a in [-1, 0, 1]:
            for b in [-1, 0, 1]:
                dKp = a * self.dKp
                dKi = b * self.dKi
                actions.append((dKp, dKi))
        return actions
    
    def enumerate_actions_2(self):
    
        ### Génération des 25 actions possibles (+/=/- dKp1 ou +/=/- dkp2, +/=/- dKi1 ou +/=/- dKi2)

        actions = [
            ( self.dKp,  self.dKi),
            ( self.dKp,  self.dKi2),
            ( self.dKp, -self.dKi),
            ( self.dKp, -self.dKi2),
            ( self.dKp, 0.0),

            ( self.dKp2,  self.dKi),
            ( self.dKp2,  self.dKi2),
            ( self.dKp2, -self.dKi),
            ( self.dKp2, -self.dKi2),
            ( self.dKp2, 0.0),

            (-self.dKp,  self.dKi),
            (-self.dKp,  self.dKi2),
            (-self.dKp, -self.dKi),
            (-self.dKp, -self.dKi2),
            (-self.dKp, 0.0),

            (-self.dKp2,  self.dKi),
            (-self.dKp2,  self.dKi2),
            (-self.dKp2, -self.dKi),
            (-self.dKp2, -self.dKi2),
            (-self.dKp2, 0.0),

            (0.0,  self.dKi),
            (0.0,  self.dKi2),
            (0.0, -self.dKi),
            (0.0, -self.dKi2),
            (0.0, 0.0),
        ]

        


    
    

    

    #def next_action(self, observation):

        # Ici c'est à creuser vu que mon entrainement a 2 phases :
        ## Phase 1 : echauffement des poids avec P2 (donc indépendamment des actions du dessus)
        ## Phase 2 : entrainement VFA/RLSTD avec epsilon-greedy (soit VFA, soit action random)




