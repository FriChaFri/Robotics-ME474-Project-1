import mujoco
import mujoco.viewer
import time

# Load the XML model
model = mujoco.MjModel.from_xml_path("notebooks\penulum.xml")
data = mujoco.MjData(model)

# Open the MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_FRAME] = True  # Show frames
        mujoco.mj_step(model, data)  # Step the simulation
        viewer.sync()  # Update the viewer
        time.sleep(0.01)  # Add delay to match real-time

