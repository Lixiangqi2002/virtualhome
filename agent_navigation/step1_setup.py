import sys
import os
import time

# Add current directory to path just in case
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from navigator import AgentNavigator

exec_path = '/data/code/virtualhome/virtualhome/simulation/unity_simulator/linux_exec.v2.3.0.x86_64'
output_dir = '/data/code/virtualhome/agent_navigation/output'
os.makedirs(output_dir, exist_ok=True)

print("Starting Step 1 Setup")
nav = AgentNavigator(exec_path=exec_path)

try:
    if nav.reset(0):
        print("Reset Successful")
        if nav.add_agent():
            print("Agent and Camera added")
            video_path = os.path.join(output_dir, 'step1.mp4')
            nav.start_video(video_path)
            
            for i in range(20):
                print(f"Recording frame {i}...")
                pos_rot = nav.get_position()
                if pos_rot:
                    pos, rot = pos_rot
                    pos_str = f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                    rot_str = f"Rot: ({rot[0]:.2f}, {rot[1]:.2f}, {rot[2]:.2f})"
                else:
                    pos_str = "Pos: Unknown"
                    rot_str = "Rot: Unknown"
                
                nav.add_frame([
                    f"Frame: {i}",
                    pos_str,
                    rot_str
                ])
                time.sleep(0.05)

            nav.stop_video()
            print(f"Step 1 Complete! Video saved to {video_path}")
        else:
            print("Failed to add agent")
    else:
        print("Failed to reset")
finally:
    nav.close()
