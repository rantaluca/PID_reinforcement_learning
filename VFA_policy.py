import sys866_lib_Windows as lib
import numpy as np
import os

class RLSTDPolicy(lib.Policy):



    def __init__(
        self, 
        consigne, 
        consigne_tresh=0.98, 
        loss_params={'error':0.5, 'dep':10, 'conv':0.5}, 
        dt=0.1,
        # Références pour la normalisation des gains
        Kp_ref = 1.0,
        Ki_ref = 0.1,
        # Pas
        dKp = 0.01,
        dKi = 0.001,
        dKp2 = 0.05,
        dKi2 = 0.005,
        #Limites
        Kp_bounds=(0.0, 1.0),
        Ki_bounds=(0.0, 1.0),
        # Paramètres RLSTD
        beta = 100,
        epsilon = 0.1,
        gamma = 0.99,
        Integral_ref = 1.0, # Au plus ce facteur est élevé, au plus l'erreur intégrale normalisée pénalise
        # Gains initiaux (P2)
        Kp_init = 0.4,
        Ki_init = 0.04,
    ):

        super().__init__(
            consigne = consigne, 
            consigne_tresh = consigne_tresh, 
            loss_params = loss_params, 
            dt = dt)
    
        self.Kp = Kp_init
        self.Ki = Ki_init
        self.Kd = 0.0

        self.Kp_ref = Kp_ref
        self.Ki_ref = Ki_ref

        self.dKp = dKp
        self.dKi = dKi
        self.dKp2 = dKp2
        self.dKi2 = dKi2

        self.Kp_bounds = Kp_bounds
        self.Ki_bounds = Ki_bounds

        self.beta = beta
        self.epsilon = epsilon
        self.gamma = gamma

        self.prev_error = 0.0
        self.cumulative_error = 0.0
        self.Integral_ref = Integral_ref

        # Variables RLSTD
        self.number_of_features = 8
        self.theta = np.zeros(self.number_of_features)
        self.A = np.eye(self.number_of_features) * self.beta
        self.last_phi = None
        self.last_reward = 0.0

        self.actions_set = 1
        self.train_value_function = True
        self.adapt_gains = True


    
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
    


    def update_value_function(self, phi_next):
        
        ### Algorithme RLSTD

        ## Première itération

        if self.last_phi is None:
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

        # Mise à jour theta
        self.theta = theta + k * delta

        #Mise à jour A
        g_col = A @ u
        self.A = A - np.outer(k, g_col)

        #Mise à jour mémoire
        self.last_phi = phi_next.copy()
        


    def compute_loss(self, observation):

        ### Fonction de coût via classe parente

        current_loss, running_loss = super().compute_loss(observation)

        self.last_reward = -current_loss

        return current_loss, running_loss



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

        return actions



    def next_action(self, observation):

        consigne, error, command, output = observation

        if self.actions_set == 0:
            actions = self.enumerate_actions()
        else:
            actions = self.enumerate_actions_2()

        #Politique epsilon-greedy basée sur équation de Bellman
            
        # Mise à jour de l'erreur cumulative

        if consigne != 0:
            self.cumulative_error += (error / consigne) * self.dt
        
        # Calcul des features du nouveau état

        phi_current = self.compute_features(observation)

        # Mise à jour de la VFA si activée

        if self.train_value_function:
            self.update_value_function(phi_current)
        
        # Calcul de la perte actuelle

        c_t = self.current_loss

        #Phase 1 : Entrainement RLSTD sur P2
        
        if not self.adapt_gains:

            return self.Kp, self.Ki, self.Kd
            
        #Phase 2 : Politique epsilon-greedy
            # Si epsilon vaut 0, politique normale

        if np.random.rand() < self.epsilon:
            #Exploration
            dKp, dKi = actions[np.random.randint(len(actions))]
        else:
            #Exploitation
            best_score = np.inf
            best_dKp, best_dKi = 0.0, 0.0

            for dKp_cand, dKi_cand in actions:

                Kp_cand = np.clip(self.Kp + dKp_cand, *self.Kp_bounds)
                Ki_cand = np.clip(self.Ki + dKi_cand, *self.Ki_bounds)

                phi_cand = phi_current.copy()

                phi_cand[6] = Kp_cand / self.Kp_ref
                phi_cand[7] = Ki_cand / self.Ki_ref

                V_cand = float(self.theta @ phi_cand)

                score = c_t + self.gamma * V_cand

                if score < best_score:
                    best_score = score
                    best_dKp, best_dKi = dKp_cand, dKi_cand

            dKp, dKi = best_dKp, best_dKi

        self.Kp = float(np.clip(self.Kp + dKp, *self.Kp_bounds))
        self.Ki = float(np.clip(self.Ki + dKi, *self.Ki_bounds))
        self.Kd = 0.0

        return self.Kp, self.Ki, self.Kd



    def save_params(self, filepath):
        
        # Sauvegarde de theta et A
        
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        np.savez(filepath, theta=self.theta, A=self.A)
        print(f"[RLSTDPolicy] Paramètres sauvegardés dans {filepath}")
    

    
    def load_params(self, filepath):
        
        # Chargement de theta et A

        data = np.load(filepath)
        theta_loaded = data["theta"]
        A_loaded = data["A"]

        assert theta_loaded.shape == self.theta.shape
        assert A_loaded.shape == self.A.shape

        self.theta = theta_loaded.copy()
        self.A = A_loaded.copy()

        self.last_phi = None

        print(f"[RLSTDPolicy] Paramètres chargés depuis {filepath}")
    
    def reset_episode(self):

        # Enchainer plusieurs simulations en conservant paramètres, et en reinitialisant le reste

        self.running_loss = 0.0
        self.current_loss = 0.0

        self.prev_error = 0.0
        self.cumulative_error = 0.0

        self.last_phi = None
        self.last_reward = 0.0

        



        







