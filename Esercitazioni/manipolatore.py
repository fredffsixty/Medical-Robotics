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
            self.__dh[joint[0] - 1]['parametri'][2 if self.__dh[joint[0] - 1] == 'prismatico' else 3] = joint[1]