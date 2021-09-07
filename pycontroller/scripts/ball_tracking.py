import math
from simple_pid import PID

pid_x = PID(0.1, 0.5, 2.0, setpoint=0)
pid_y = PID(0.1, 0.5, 2.0, setpoint=0)

# stall movement

pid_x.output_limits = (-1.0, 1.0)
pid_y.output_limits = (-1.0, 1.0)

pitch = 0.0
yaw = 0.0

errorPitch = 0.0
out_scale = 0.5

ball_track = None

isEnabled = False

def track(error):
    global pid_x
    global yaw
    global pid_y
    global pitch

    ex = error.x
    if(ex > 0.1 or ex < -0.1):
        ex -= 0.11

    out_x = (ex) * out_scale
    out_y = pid_y(error.y) * out_scale
    out_y = 0.0

    # print(out_x)

    pitch += out_y
    yaw += max(min(-out_x, 0.1), -0.1)

    pitch = max(min(pitch, 1), -1)
    yaw = max(min(yaw, 1), -1)