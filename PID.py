import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# -----------------------------------------
# Thermal Plant Parameters
# -----------------------------------------

Tamb = 25.0          # Ambient temperature
tau = 80.0           # Thermal time constant
heater_gain = 3.0

dt = 0.5
t_final = 600

# -----------------------------------------
# Initial PID Parameters
# -----------------------------------------

kp0 = 2.0
ki0 = 0.03
kd0 = 10.0

# -----------------------------------------
# Figure
# -----------------------------------------

fig, ax = plt.subplots(figsize=(10,6))
plt.subplots_adjust(left=0.15,bottom=0.35)

time = np.arange(0,t_final,dt)

# -----------------------------------------
# Simulation Function
# -----------------------------------------

def simulate(kp,ki,kd,setpoint):

    T = Tamb

    integral = 0
    previous_error = 0

    T_history = []

    for t in time:

        error = setpoint - T

        integral += error*dt

        derivative = (error-previous_error)/dt

        u = (
            kp*error
            + ki*integral
            + kd*derivative
        )

        u = max(0,min(100,u))

        dTdt = (
            heater_gain*u/100
            - (T-Tamb)/tau
        )

        T += dTdt*dt

        previous_error = error

        T_history.append(T)

    return np.array(T_history)

# -----------------------------------------
# Initial Plot
# -----------------------------------------

setpoint0 = 100

response = simulate(kp0,ki0,kd0,setpoint0)

line, = ax.plot(time,response,
                lw=2,
                label='Temperature')

sp_line = ax.axhline(
    setpoint0,
    color='red',
    linestyle='--',
    label='Setpoint'
)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Temperature (°C)")
ax.set_title("ENGY 300 PID Thermal Playground")
ax.grid(True)
ax.legend()

# -----------------------------------------
# Sliders
# -----------------------------------------

ax_kp = plt.axes([0.15,0.22,0.7,0.03])
ax_ki = plt.axes([0.15,0.17,0.7,0.03])
ax_kd = plt.axes([0.15,0.12,0.7,0.03])
ax_sp = plt.axes([0.15,0.07,0.7,0.03])

s_kp = Slider(ax_kp,'Kp',0,20,valinit=kp0)
s_ki = Slider(ax_ki,'Ki',0,1,valinit=ki0)
s_kd = Slider(ax_kd,'Kd',0,40,valinit=kd0)
s_sp = Slider(ax_sp,'Setpoint',30,200,valinit=setpoint0)

# -----------------------------------------
# Update Function
# -----------------------------------------

def update(val):

    response = simulate(
        s_kp.val,
        s_ki.val,
        s_kd.val,
        s_sp.val
    )

    line.set_ydata(response)

    sp_line.set_ydata(
        [s_sp.val,s_sp.val]
    )

    fig.canvas.draw_idle()

s_kp.on_changed(update)
s_ki.on_changed(update)
s_kd.on_changed(update)
s_sp.on_changed(update)

# -----------------------------------------
# Disturbance Button
# -----------------------------------------

resetax = plt.axes([0.85,0.9,0.1,0.05])

button = Button(
    resetax,
    'Reset'
)

def reset(event):

    s_kp.reset()
    s_ki.reset()
    s_kd.reset()
    s_sp.reset()

button.on_clicked(reset)

plt.show()