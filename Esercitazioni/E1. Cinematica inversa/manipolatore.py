"""
Modulo che esporta la definizione di un manipolatore robotico
"""

import numpy as np
import jax.numpy as jnp
from jax import jacfwd, Array
from scipy.optimize import least_squares

def to_ndarray(jnparray, shape=(1,)):
    
    l = []
    for el in jnparray.ravel():
        l.append(float(el))
    
    v = np.array(l)
    
    return v.reshape(shape)
    

class Manipolatore:
    """
    Manipolatore: classe che implementa la cinematica e lo Jacobiano
    di un manipolatore ad n giunti prismatici o di rotazione
    """
    
    def __init__(self, dh, bounds):
        """
            Costruisce il manipolatore a partire dalla tabella di Denavit Hartenberg
            costruita come una lista di dizionari:

            [{"parametri":[alpha_i, a_i, d_i, theta_i], "tipo":"rotazione"|"prismatico"},
            ...]

            Ovvero da un iterabile di coppie, ad esempio:

            (([alpha1, a1, d1, theta1],"prismatico"),([alpha2, a2, d2, theta2],"rotazione"),...)
            
            Richiede anche una lista di coppie di estremi dei valori dei parametri:
            
            [(lower_1,upper_1), ..., (lower_n,uper_n)] o None se non sono specificati
            
            Se alcuni parametri non sono limitati usare np.inf o -np.inf come valori estremi
        """

        self.__dh = []
        for joint in dh:
            self.__dh.append(\
                joint if isinstance(dh,(list, tuple)) and isinstance(dh[0],dict)\
                else dict((('parametri',joint[1]),('tipo',joint[0])))\
            )
        
        self.bounds = bounds
        
    def __repr__(self):
        joints = '['
        
        for joint in self.__dh:
            joints = joints + \
                (f"({joint['tipo']},{joint['parametri'][2]})" if self.__dh.index(joint) == -1 \
                else f"({joint['tipo']},{joint['parametri'][2]}),")
                
        return joints + ']'
    
    # Creiamo la coppia get/set per la lista dei giunti
    @property   
    def joints(self):
        # restituisce la lista di coppie (tipo, parametro) per tutti i giunti
        return [(joint['tipo'],\
                joint['parametri'][2 if joint['tipo'] == 'prismatico' else 3])\
                for joint in self.__dh]
    
    @joints.setter
    def joints(self, joints):
        # accetta un iterabile di coppie (posizione, valore) per cui modifica 
        # il valore del parametro che si trova in una data posizione 
        # nella lista dei giunti.
        #
        # La posizione viene fornita convenzionalmente a partire da 1
        
        for joint in joints:
            self._Manipolatore__dh[joint[0] - 1]['parametri']\
                [2 if self._Manipolatore__dh[joint[0] - 1]['tipo'] == 'prismatico' else 3] = joint[1]
    
    # Metodo per ottenere la lista dei valori dei parametri
    def get_parameters(self):
        
        return [x[1] for x in self.joints]

    # Metodo "nascosto" per calcolare la matrice al singolo giunto
    # 
    # Riceve in ingresso il parametro e l'indice i del giunto 
    # (partendo da 1) e restituisce
    # l'array numpy corrsipondente alla matrice Mi-1,i
    def _m_joint(self, param, joint_index):
        
        matrix = np.eye(4,dtype='float')
        
        # Assegniamo i valori dei parametri a delle variabili di comodo
        alpha_i = self._Manipolatore__dh[joint_index - 1]['parametri'][0]
        a_i = self._Manipolatore__dh[joint_index - 1]['parametri'][1]
        d_i = param if self._Manipolatore__dh[joint_index - 1]['tipo'] == 'prismatico'\
            else self._Manipolatore__dh[joint_index - 1]['parametri'][2]
        theta_i = param if self._Manipolatore__dh[joint_index - 1]['tipo'] == 'rotazione'\
            else self._Manipolatore__dh[joint_index - 1]['parametri'][3]
        
        # Inseriamo d_i in posizione (2,3) se diverso da zero
        if d_i != 0.0:
            matrix[2, 3] = d_i
            #matrix = matrix.at[2, 3].set(d_i) # assegnamento JAX
        
        # Calcolo efficiente di sin e cos di alpha_i e theta_i
        if alpha_i == 0.0:
            sin_alpha_i = 0
            cos_alpha_i = 1
        elif alpha_i == 90.0:
            sin_alpha_i = 1
            cos_alpha_i = 0
        elif alpha_i == -90.0:
            sin_alpha_i = -1
            cos_alpha_i = 0
        elif alpha_i == 180.0 or alpha_i == -180.0:
            sin_alpha_i = 0
            cos_alpha_i = -1
        else:
            sin_alpha_i = jnp.sin(jnp.deg2rad(alpha_i))
            cos_alpha_i = jnp.cos(jnp.deg2rad(alpha_i))

        if theta_i == 0:
            sin_theta_i = 0
            cos_theta_i = 1
        elif theta_i == 90.0:
            sin_theta_i = 1
            cos_theta_i = 0
        elif theta_i == -90.0:
            sin_theta_i = -1
            cos_theta_i = 0
        elif theta_i == 180.0 or theta_i == -180.0:
            sin_theta_i = 0
            cos_theta_i = -1
        else:
            sin_theta_i = jnp.sin(jnp.deg2rad(theta_i))
            cos_theta_i = jnp.cos(jnp.deg2rad(theta_i))
        
        # Inseriamo i coefficienti nella matrice
        #matrix = matrix.at[0, 0].set(cos_theta_i)
        #matrix = matrix.at[1, 0].set(sin_theta_i)
        #matrix = matrix.at[2, 1].set(sin_alpha_i)
        #matrix = matrix.at[2, 2].set(cos_alpha_i)

        matrix[0, 0] = cos_theta_i
        matrix[1, 0] = sin_theta_i
        matrix[2, 1] = sin_alpha_i
        matrix[2, 2] = cos_alpha_i
        
        #matrix = matrix.at[0, 1].set(-cos_alpha_i*sin_theta_i)
        #matrix = matrix.at[0, 2].set(sin_alpha_i*sin_theta_i)
        #matrix = matrix.at[1, 1].set(cos_alpha_i*cos_theta_i)
        #matrix = matrix.at[1, 2].set(-sin_alpha_i*cos_theta_i)

        matrix[0, 1] = -cos_alpha_i*sin_theta_i
        matrix[0, 2] = sin_alpha_i*sin_theta_i
        matrix[1, 1] = cos_alpha_i*cos_theta_i
        matrix[1, 2] = -sin_alpha_i*cos_theta_i
        
        if a_i != 0:
            #matrix = matrix.at[0, 3].set(a_i*cos_theta_i)
            #matrix = matrix.at[1, 3].set(a_i*sin_theta_i)
            matrix[0, 3] = a_i*cos_theta_i
            matrix[1, 3] = a_i*sin_theta_i
        
        return matrix
    
    def forward_kinematics(self, params):
        # Calcola la cinematica diretta del manipolatore
        # partendo dalla lista dei parametri ai giunti
        
        # matrice identità che conterrà la cinematica diretta
        fk = np.eye(4,dtype='float')
        
        # itero lungo la lista dei giunti della tabella DH
        # e accumulo i prodotti interni della matrice fk con la
        # trasformazione M_i-1,i
        for i in range(len(self._Manipolatore__dh)):
            fk = np.matmul(fk,self._m_joint(params[i],i+1))
        
        return fk
    
    def jacobiano(self, params):
        
        def fun(x):
            
            # calcoliamo la cinematica diretta

            par = to_ndarray(x, shape=(len(x),))
            fk = self.forward_kinematics(par)

            res = np.array(
                [fk[0,3], fk[1,3], fk[2,3], 
                 fk[0,0], fk[0,1], fk[0,2],
                 fk[1,0], fk[1,1], fk[1,2],
                 fk[2,0], fk[2,1], fk[2,2]])
            
            return jnp.asarray(res)
        
        p = jnp.asarray(params)
        jac = jacfwd(fun)(p)
        
        return to_ndarray(jac,shape=jac.shape)
        
    def inverse_kinematics(self, position, start_value=None, orientation=None, regularization=0.5):
        """Cinematica inversa

        Args:
            position (List[float]): se orientation = 'all', la posizione e orientazione del target [p_T, n_T, o_T, a_T];
                se orientation = 'X'|'Y'|'Z', la posizione [p_T, axis] dove axis è l'asse 
                che individua la direzione X, Y, o Z al target
            
            start_value (List[float], optional): lista dei parametri estratta dai sensori. Defaults to None.
            
            orientation (str, optional): orientamento dell'effettore al target: 'X'|'Y'|'Z'|'all'. Defaults to None.
            
            regularization (float, optional): parametro di regolarizzazione. Defaults to 0.5.

        Returns:
            ndarray shape (n,): lista dei parametri che risolve la cinematica inversa
        """
        if start_value != None:
            params = start_value
        else:
            # estraggo la lista dei valori parametri
            params = [x[1] for x in self.joints]
        
        # calcolo il vettore degli estremi
        if self.bounds != None:
            bounds = ([x[0] for x in self.bounds],[x[1] for x in self.bounds])
        else:
            bounds = (-np.inf, np.inf)
        
        # definisco la funzione obiettivo
        def target_fun(params, position, orientation):
            
            # calcoliamo la cinematica diretta
            fk = self.forward_kinematics(params)

            # costruiamo il target di posizione
            target = np.asarray(position[0:3]) - fk[:3,3]

            # costruiamo il target di orientazione come matrice identità
            target_or = np.eye(3, dtype=float)

            # Condizioniamo le colonne del target di orientazione a seconda
            # della richiesta di orientamento dell'approccio dell'effettore
            if orientation == 'X':
                target_or[:3,0] = position[3:6]
                #target_or = target_or.at[:3,0].set(position[3:6])
            elif orientation == 'Y':
                target_or[:3,1] = position[3:6]
                #target_or = target_or.at[:3,1].set(position[3:6])
            elif orientation == 'Z':
                target_or[:3,2] = position[3:6]
                #target_or = target_or.at[:3,2].set(position[3:6])
            elif orientation == 'all':
                target_or = position[3:12]

            if orientation != None:

                # creiamo il vettore differenza tra la posizione del target
                # e la versione vettorizzata della cinematica diretta
                target = np.concatenate((target, target_or.ravel()))

            # restituiamo la funzione f(p) = reg*target^2
            return regularization * np.linalg.norm(target)
        
        # invochiamo la routine di ottimizzazione
        res = least_squares(target_fun, params, args=(position, orientation), bounds=bounds)
        
        return res.x
    