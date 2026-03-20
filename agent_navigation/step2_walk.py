import sys
import os
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from navigator import AgentNavigator

exec_path = '/data/code/virtualhome/virtualhome/simulation/unity_simulator/linux_exec.v2.3.0.x86_64'
output_dir = '/data/code/virtualhome/agent_navigation/output'
os.makedirs(output_dir, exist_ok=True)

print("Starting Step 2 Walking & Tracking")
nav = AgentNavigator(exec_path=exec_path)

try:
    if nav.reset(0):
        print("Reset Successful")
        if nav.add_agent():
            print("Agent and Camera added")
            video_path = os.path.join(output_dir, 'step2_walk.mp4')
            nav.start_video(video_path, dual_view=True)
            
            # Get starting position
            pos_rot = nav.get_position()
            if not pos_rot:
                print("Failed to get starting position")
                sys.exit(1)
            
            pos, rot = pos_rot
            print(f"Starting Position: {pos}")
            
            # Simulate walks: move forward incrementally in X-direction
            current_pos = list(pos) # copy as list [x, y, z]
            
            # 20 steps, step size 0.1m along X
            step_size = 0.1 
            for step in range(30):
                # Update position
                current_pos[0] += step_size 
                
                # Apply movement using the updated method that ticks the frame
                success = nav.move_character(current_pos)
                if not success:
                    print(f"Failed to move character on step {step}")
                    break
                
                # Verify position from graph
                actual_pos_rot = nav.get_position()
                if actual_pos_rot:
                    actual_pos = actual_pos_rot[0]
                    print(f"Step {step}: Commanded X: {current_pos[0]:.2f}, Actual X: {actual_pos[0]:.2f}")
                    
                    pos_str = f"Pos: ({actual_pos[0]:.2f}, {actual_pos[1]:.2f}, {actual_pos[2]:.2f})"
                else:
                    pos_str = "Pos: Unknown"
                    
                nav.add_frame([
                    f"Step: {step}",
                    f"Commanded X: {current_pos[0]:.2f}",
                    pos_str
                ])
                time.sleep(0.05)

            nav.stop_video()
            print(f"Step 2 Complete! Video saved to {video_path}")
        else:
            print("Failed to add agent")
    else:
        print("Failed to reset")
finally:
    nav.close()
