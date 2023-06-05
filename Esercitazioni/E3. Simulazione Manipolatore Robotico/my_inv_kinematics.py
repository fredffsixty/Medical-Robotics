from controller import Supervisor
from manipolatore import Manipolatore
import numpy as np

# Creiamo il robot in Webots e acquisiamo il time step di defalut
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# accediamo ai nodi del robot, del target e della sua geometria
arm = robot.getSelf()
target = robot.getFromDef('TARGET')
targetShape = robot.getFromDef('TARGET_SHAPE')

# conserviamo le dimensioni del target in due
# variabili di comodo
radius = targetShape.getField('radius').value
height = targetShape.getField('height').value

# Creiamo la lista dei motori accedendo ai singoli motori tramite il nome con
# cui sono stati definiti nel PROTO
motors = [robot.getDevice(name) for name in ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']]

for motor in motors:
    motor.setVelocity(1.0)     # velocità costante: non necessitiamo dello Jacobiano geometrico
    position_sensor = motor.getPositionSensor()
    position_sensor.enable(timestep)

# estraiamo i limiti di variazione dai motori
bounds = [(np.rad2deg(motor.getMinPosition()),np.rad2deg(motor.getMaxPosition())) for motor in motors]

# correggiamo secondo la tabella di DH
bounds[1] = (bounds[1][0]-90.0,bounds[1][1]-90.0)
bounds[4] = (bounds[4][0]+180.0,bounds[4][1]+180.0)


# creiamo il maniplatore IRB4600-40 in configurazione DH
irb = Manipolatore(
                ({'tipo':'rotazione','parametri':[-90.0, 0.175, 0.495, 0.0]},
                {'tipo':'rotazione','parametri':[0.0, 1.095, 0.0, -90.0]},  # theta_1 - 90° in tabella DH
                {'tipo':'rotazione','parametri':[-90.0, 0.175, 0.0, 0.0]},
                {'tipo':'rotazione','parametri':[90.0, 0.0, 1.27, 0.0]},
                {'tipo':'rotazione','parametri':[90.0, 0.0, 0.0, 180.0]},   # theta_5 + 180° in tabella DH
                {'tipo':'rotazione','parametri':[0.0, 0.0, 0.135, 0.0]}),
                bounds
                )


# Main loop:
while robot.step(timestep) != -1:
    # Acquisiamo le posizioni di effettore e target
    # nel sistema di riferimento assoluto 
    targetPosition = target.getPosition()
    
    armPosition = arm.getPosition() # posizione della base e "non" dell'effettore
    
    # Posizione del target rispetto al riferimento del braccio
    targetPosition[0] = targetPosition[0] - radius - armPosition[0] # l'effettore si appoggia alla superficie
    targetPosition[1] = targetPosition[1] - armPosition[1]
    targetPosition[2] = targetPosition[2] - armPosition[2]
    
    orientation = target.getOrientation()

    # Calcoliamo esplicitamente la cinematica inversa
    # la configurazione dei giunti è quella che ci restituiscono i 
    # sensori sul robot
    initial_position = [np.rad2deg(m.getPositionSensor().getValue()) for m in motors]
    
    # correzione angoli per tabella DH IRB4600-40
    initial_position[1] -= 90.0
    initial_position[4] += 180.0
    
    ikResults = irb.inverse_kinematics(targetPosition + [orientation[0], orientation[3], orientation[6]],
                                        orientation='X',
                                        start_value=initial_position,
                                        regularization=0.1)

    # correzione inversa angoli per tabella DH IRB4600-40
    ikResults[1] += 90.0
    ikResults[4] -= 180.0
    
    # Calcoliamo la nuova posizione dai giunti con la cimematica diretta
    position = irb.forward_kinematics(ikResults)
    
    # Attuiamo i motori con i nuvi valori dei giunti
    for i in range(len(motors)):
        motors[i].setPosition(np.deg2rad(ikResults[i]))

        






