import mujoco
import mujoco.viewer
import time
import numpy as np

# Load the XML model
model = mujoco.MjModel.from_xml_path("notebooks\penulum.xml")
data = mujoco.MjData(model)


joint1_name = "joint1"
joint2_name = "joint2"

joint1_id = model.joint(joint1_name).id
joint2_id = model.joint(joint2_name).id



#Verify 1A



# Open the MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    end_effector_body_name="end-effector"
    end_effector_body_id = model.body(name=end_effector_body_name).id
    while viewer.is_running():
        # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_FRAME] = True  # Show frames
        mujoco.mj_step(model, data)  # Step the simulation
        viewer.sync()  
        time.sleep(0.01)
        endeffector_pos = data.xpos[end_effector_body_id]

        theta1 = data.qpos[joint1_id]
        theta2 = data.qpos[joint2_id]

        print(f"Theta1: {round(np.degrees(theta1), 2)}° Theta2: {round(np.degrees(theta2), 2)}°")


