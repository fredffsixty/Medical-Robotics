"""ik controller."""

import math
import tempfile
from controller import Supervisor

# Useremo il package ikpy per la gestione della cinematica inversa
# qui potremmo importare la nostra classe di definizione del manipolatore
import ikpy
from ikpy.chain import Chain

# Creiamo l'istanza del Supervisor
robot = Supervisor()

# Definiamo uno step temporale per il loop degli eventi
timeStep = int(4 * robot.getBasicTimeStep())

# Creiamo un file temporaneo in formato URDF (Universal Robot description Format)
# che definisce una catena di link in cui i giunti connettono
# un link "padre" e un link "figlio" nel senso del sistema di riferimento.
# La libreria di cinematica inversa ikpy legge questo formato
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(robot.getUrdf().encode('utf-8'))

# Creiamo la catena cinematica: il link di base è fisso e quindi non "attivo"
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True])

# Inizializziamo i motori creando la lista dei componenti della
# catena cinematica in cui compare la stringa 'motor' questi saranno i giunti
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = robot.getDevice(link.name)
        motor.setVelocity(1.0)     # velocità costante: non necessitiamo dello Jacobiano geometrico
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

# Acquisiamo il riferimento al nodo target e al nodo robot del nostro mondo WeBots
target = robot.getFromDef('TARGET')
arm = robot.getSelf()
targetShape = robot.getFromDef('TARGET_SHAPE')

# conserviamo le dimensioni del target in due
# variabili di comodo
radius = targetShape.getField('radius').value
height = targetShape.getField('height').value

# CICLO DI CONTROLLO DI UN ROBOT
# Ad ogni time step di simulazione:
#         lettura dei sensori
#         processing dei dati
#         scrittura sugli attuatori
while robot.step(timeStep) != -1:
    # Acquisiamo le posizioni di effettore e target
    # nel sistema di riferimento assolluto 
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()
    
    # Posizione del target rispetto al braccio
    x = targetPosition[0] - radius - armPosition[0] # l'effettore si appoggia alla superficie
    y = targetPosition[1] - armPosition[1]
    z = targetPosition[2] - armPosition[2]
    
    orientation = target.getOrientation()
    
    
    # Calcoliamo esplicitamente la cinematica inversa
    # la prima configurazione dei giunti è quella che ci restituiscono i 
    # sensori sul robot
    # concateniamo un valore nullo in relazione al primo link che è fisso
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    ikResults = armChain.inverse_kinematics([x, y, z],\
                                [orientation[0], orientation[3], orientation[6]],\
                                orientation_mode='X',\
                                initial_position=initial_position,\
                                regularization_parameter=0.01)

    # Calcoliamo la nuova posizione dai giunti con la cimematica diretta
    position = armChain.forward_kinematics(ikResults)
    
    # Attuiamo i motori con i nuvi valori dei giunti
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])
