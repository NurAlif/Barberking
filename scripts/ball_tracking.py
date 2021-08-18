import math
from simple_pid import PID

pid_x = PID(2, 0.5, 0.05, setpoint=0)
pid_y = PID(2, 0.5, 0.05, setpoint=0)

pid_x.output_limits = (-1.0, 1.0)
pid_y.output_limits = (-1.0, 1.0)

pitch = 0.0
yaw = 0.0

errorPitch = 0.0
out_scale = 1.0

ball_track = None

isEnabled = False

def track(error):
    global pid_x
    global yaw
    global pid_y
    global pitch

    print(str(error.x) + " " + str(error.y))

    out_x = pid_x(error.x) * out_scale
    out_y = pid_y(error.y) * out_scale
    out_y = 0.0

    pitch = max(min(out_y, 1), -1)
    yaw = max(min(out_x, 1), -1)