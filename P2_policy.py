import numpy as np
import sys866_lib as lib

class P2Policy(lib.Policy):
  def __init__(self,
               consigne, 
               consigne_tresh=0.97,
               loss_params={'error':1.0, 'dep':0.5, 'conv':0.3},
               dt=0.1,
               Kp_ref =5.0,
               Ki_ref =1.0,
               Kd_ref =1.0,
               beta=100,
               epsilon=0.1,
               gamma =0.99,
               Integral_ref=1,
              ):

                super().__init__(
                  consigne,
                  consigne_tresh,
                  loss_params,
                  dt)
                self.Kp_ref=Kp_ref
                self.Ki_ref=Ki_ref
                self.Kd_ref=Kd_ref
                self.beta= beta
                self.epsilon=epsilon
                self.gamma = gamma
                self.prev_error = 0
                self.Integral_ref= Integral_ref
                

    
    def compute_features(self, observation):

        ## Calcul des features pour la VFA (phi)

        #1 Le biais vaut 1

        bias = 1.0


        

        #789 Gains normalisés

        Kp_norm = self.Kp / self.Kp_ref
        Ki_norm = self.Ki / self.Ki_ref
        Kd_norm = self.Kd / self.Kd_ref

        phi = np.array(
            Kp_norm,
            Ki_norm,
            Kd_norm
        ], dtype=float)

        #On garde l'erreur actuelle pour le prochain step

        self.prev_error = error

        return phi
    




    def update_value_function(self, phi_next):
        
        
        self.last_phi = phi.copy()
        


    
  



    #def next_action(self, observation):

        # Ici c'est à creuser vu que mon entrainement a 2 phases :
        ## Phase 1 : echauffement des poids avec P2 (donc indépendamment des actions du dessus)
        ## Phase 2 : 
