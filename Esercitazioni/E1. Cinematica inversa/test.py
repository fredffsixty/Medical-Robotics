from manipolatore import Manipolatore

robot = Manipolatore(
                ({'tipo':'rotazione','parametri':[-90.0, 0.175, 0.495, 60.0]},
                {'tipo':'rotazione','parametri':[0.0, 1.095, 0.0, -55.0]},
                {'tipo':'rotazione','parametri':[-90.0, 0.175, 0.0, 45.0]},
                {'tipo':'rotazione','parametri':[90.0, 0.0, 1.27, 45.0]},
                {'tipo':'rotazione','parametri':[90.0, 0.0, 0.0, 30.0]},
                {'tipo':'rotazione','parametri':[0.0, 0.0, 0.135, 45.0]}),
                [(-180.0,180.0),
                 (-90.0,150.0),
                 (-180.0,75.0),
                 (-400.0,400.0),
                 (-125.0,120.0),
                 (-400.0,400.0)]
                )


params = robot.get_parameters()

print(params)

robot.jacobiano(params)
#fk = robot.forward_kinematics(params)
#
#print(fk)
#
#pp = robot.inverse_kinematics(fk[:3,3],orientation='all')
#
#print(pp.x)
