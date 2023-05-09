"""
Modulo che esporta la definizione di un manipolatore robotico
"""

import numpy as np

class Manipolatore:
    """
    Manipolatore: classe che implementa la cinematica e lo Jacobiano
    di un manipolatore ad n giunti prismatici o di rotazione
    """
    
    def __init__(self, dh):
        """
            Costruisce il manipolatore a partire dalla tabella di Denavit Hartenberg
            costruita come una lista di dizionari:

            [{"parametri":[alpha_i, a_i, d_i, theta_i], "tipo":"rotazione"|"prismatico"},
            ...]

            Ovvero da un iterabile di coppie, ad esempio:

            (([alpha1, a1, d1, theta1],"prismatico"),([alpha2, a2, d2, theta2],"rotazione"),...)
        """

        self.__dh = []
        for joint in dh:
            self.__dh.append(\
                joint if isinstance(dh,(list, tuple)) and isinstance(dh[0],dict)\
                else dict((('parametri',joint[1]),('tipo',joint[0])))\
            )
        
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

    # Metodo "nascosto" per calcolare la matrice al singolo giunto
    # 
    # Riceve in ingresso l'indice i del giunto (partendo da 1) e restituisce
    # l'array numpy corrsipondente alla matrice Mi-1,i
    def _m_joint(self, joint_index):
        
        matrix = np.eye(4,dtype='float')
        
        # Assegniamo i valori dei parametri a delle variabili di comodo
        alpha_i = self._Manipolatore__dh[joint_index - 1]['parametri'][0]
        a_i = self._Manipolatore__dh[joint_index - 1]['parametri'][1]
        d_i = self._Manipolatore__dh[joint_index - 1]['parametri'][2]
        theta_i = self._Manipolatore__dh[joint_index - 1]['parametri'][3]
        
        # Inseriamo d_i in posizione (2,3) se diverso da zero
        if d_i != 0.0:
            matrix[2,3] = d_i
        
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
            sin_alpha_i = np.sin(np.deg2rad(alpha_i))
            cos_alpha_i = np.cos(np.deg2rad(alpha_i))

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
            sin_theta_i = np.sin(np.deg2rad(theta_i))
            cos_theta_i = np.cos(np.deg2rad(theta_i))
        
        # Inseriamo i coefficienti nella matrice
        matrix[0, 0] = cos_theta_i
        matrix[1, 0] = sin_theta_i
        matrix[2, 1] = sin_alpha_i
        matrix[2, 2] = cos_alpha_i
        
        matrix[0, 1] = -cos_alpha_i*sin_theta_i
        matrix[0, 2] = sin_alpha_i*sin_theta_i
        matrix[1, 1] = cos_alpha_i*cos_theta_i
        matrix[1, 2] = -sin_alpha_i*cos_theta_i
        
        if a_i != 0:
            matrix[0, 3] = a_i*cos_theta_i
            matrix[1, 3] = a_i*sin_theta_i
        
        return matrix
    
    def forward_kinematics(self, joint_start=None, joint_end=None):
        # Calcola la cinematica diretta tra due sistemi di riferimento
        # qualunque tra quelli solidali ai giunti
        #
        # forward_kinematics(i) --> M_0,i cinematica diretta del giunto i
        # foreard_kinematics(i,j) --> M_i,j cinematica diretta dal giunto i al giunto j
        
        if joint_start == None and joint_end == None:
            joint_start = 0
            joint_end = len(self._Manipolatore__dh)
        elif joint_end == None:
            joint_end = len(self._Manipolatore__dh)
        elif joint_start == None:
            joint_start = 0
        
        # matrice identità che conterrà la cinematica diretta
        fk = np.eye(4,dtype='float')
        
        # itero lungo la lista dei giunti della tabella DH
        # e accumulo i prodotti interni della matrice fk con la
        # trasformazione M_i-1,i
        for joint in range(joint_start+1,joint_end+1):
            fk = np.matmul(fk,self._m_joint(joint))
        
        return fk