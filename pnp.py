import numpy as np
import math as math
from autolab_core import RigidTransform

# pip install autolab_core

# 写上用四元数表示的orientation和xyz表示的position
orientation = {'y': -0.6971278819736084, 'x': -0.716556549511624, 'z': -0.010016582945017661, 'w': 0.02142651612120239}
position = {'y': -0.26022684372145516, 'x': 0.6453529828252734, 'z': 1.179122068068349}

rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
translation = np.asarray([position['x'], position['y'], position['z']])
# 这里用的是UC Berkeley的autolab_core，比较方便吧，当然可以自己写一个fuction来计算，计算公式在https://www.cnblogs.com/flyinggod/p/8144100.html
m = RigidTransform(rotation_quaternion, translation)

r = m.rotation

# pitch, yaw, roll的计算公式
pitch = math.asin(r[2, 1])
yaw = -math.atan2(r[0, 1], r[1, 1])
roll = -math.atan2(r[2, 0], r[2, 2])

pitch_ang = pitch * 180.0 / 3.1415926
yaw_ang = -yaw * 180.0 / 3.1415926  # yaw取反
roll_ang = roll * 180.0 / 3.1415926
print('----- ji result -------')
print('pitch: %f' % pitch_ang)
print('yaw: %f' % yaw_ang)
print('roll_ang: %f' % roll_ang)