from numpy import pi, sin
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from robot.robot_model import robot_arm 

axis_color = 'lightgoldenrodyellow'

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
fig.subplots_adjust(left=0.05, bottom=0.05*6 + 0.1)

# Lav robot arm
robot_arm = robot_arm(ax)

ax.set_xlim([-200,200])
ax.set_ylim([-300, 300])
ax.set_zlim([-300, 300])

# Insæt sliders for at ændre vinklerne
sliders_ax = [fig.add_axes([0.05, 0.35 - 0.05*n, 0.65, 0.03], facecolor=axis_color) for n in range(6) ]
slider_val = [Slider(sliders_ax[n], 'ang', -180.0, 180.0, valinit=robot_arm.q[n] * 180/np.pi) for n in range(6)]

# Opdater robot udfra sliders:
def sliders_on_changed(_val):
    for i in range(6):
        robot_arm.q[i] = slider_val[i].val * np.pi/180
    _ = robot_arm.forward_kinematics()
    robot_arm.update_robot()
    fig.canvas.draw_idle()

for fnc in slider_val : 
    fnc.on_changed(sliders_on_changed)

# Add a button for resetting the parameters
reset_button_ax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
reset_button = Button(reset_button_ax, 'Reset', color=axis_color, hovercolor='0.975')
def reset_button_on_clicked(mouse_event):
    for _slider_val in slider_val:
        _slider_val.reset()
reset_button.on_clicked(reset_button_on_clicked)

plt.show()