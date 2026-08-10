"""Arrow-key teleop for supermarketbot in the MuJoCo viewer.

Run from this directory:  python3 teleop.py
Keys: Up/Down = forward/back, Left/Right = turn, Space = stop, Esc = quit viewer.
The viewer only reports key-down (no key-up), so each press latches a
command until you press another key or Space to stop.
"""
import mujoco
import mujoco.viewer

WHEEL_SPEED = 0.3  # rad/s

model = mujoco.MjModel.from_xml_path("world.xml")
data = mujoco.MjData(model)

# GLFW keycodes: Up=265, Down=264, Left=263, Right=262, Space=32
CMD = {265: (1, 1), 264: (-1, -1), 263: (-1, 1), 262: (1, -1), 32: (0, 0)}


def key_callback(keycode):
    if keycode in CMD:
        left, right = CMD[keycode]
        data.ctrl[0] = left * WHEEL_SPEED
        data.ctrl[1] = right * WHEEL_SPEED


with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
