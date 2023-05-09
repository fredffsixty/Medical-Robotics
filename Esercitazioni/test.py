from manipolatore import Manipolatore

robot = Manipolatore(({'tipo':'rotazione','parametri':[-90.0, 0.0, 0.0, 60.0]},\
                {'tipo':'rotazione','parametri':[0.0, 25.0, 0.0, 45.0]}))

print(robot.joints)

print(robot.forward_kinematics(joint_end=1),'\n',\
    robot.forward_kinematics(1,2),'\n',\
    robot.forward_kinematics())
