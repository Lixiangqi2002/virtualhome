import sys
import os
import time
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from navigator import AgentNavigator

exec_path = '/data/code/virtualhome/virtualhome/simulation/unity_simulator/linux_exec.v2.3.0.x86_64'
output_dir = '/data/code/virtualhome/agent_navigation/output'
os.makedirs(output_dir, exist_ok=True)

print("Starting Step 3 Discrete 2D Position Control")
nav = AgentNavigator(exec_path=exec_path)

try:
    if nav.reset(0):
        print("Reset Successful")
        if nav.add_agent():
            print("Agent and Camera added")
            video_path = os.path.join(output_dir, 'step3_control.mp4')
            nav.start_video(video_path, dual_view=True)
            
            # 1. Get starting position
            pos_rot = nav.get_position()
            if not pos_rot:
                print("Failed to get starting position")
                sys.exit(1)
                
            pos, rot = pos_rot
            print(f"Starting Position: {pos}")
            
            # 2. Set Target 2D coordinates (X, Z) e.g. move +1.5m in X and +1.5m in Z
            target_x = pos[0] + 1.5
            target_z = pos[2] + 1.5
            print(f"Target Position: X:{target_x:.2f}, Z:{target_z:.2f}")
            
            # Use actual fetched pos for control loop
            current_pos = list(pos)
            
            # 3. Control loop parameters
            tolerance = 0.25      # stopping tolerance distance in meters
            max_steps = 100       # safeguard against infinite loops
            step_size = 0.2       # discrete step length in meters
            
            reached = False
            for step in range(max_steps):
                dx = target_x - current_pos[0]
                dz = target_z - current_pos[2]
                distance = math.sqrt(dx**2 + dz**2)
                
                print(f"Step {step}: Distance to target = {distance:.2f}m")
                
                if distance <= tolerance:
                    print("Arrived at target location within tolerance!")
                    reached = True
                    break
                    
                # Calculate normalized step vector
                step_dx = (dx / distance) * step_size
                step_dz = (dz / distance) * step_size
                
                # Calculate next commanded position
                command_pos = list(current_pos)
                command_pos[0] += step_dx
                command_pos[2] += step_dz
                
                # Apply movement
                success = nav.move_character(command_pos)
                if not success:
                     print("Failed to issue move_character command")
                     break
                     
                actual_pos_rot = nav.get_position()
                if actual_pos_rot:
                    actual_pos = actual_pos_rot[0]
                    # Update current loop variable with actual simulator position
                    current_pos = list(actual_pos)
                    
                    actual_distance = math.sqrt((target_x - actual_pos[0])**2 + (target_z - actual_pos[2])**2)
                    pos_str = f"Pos: ({actual_pos[0]:.2f}, {actual_pos[1]:.2f}, {actual_pos[2]:.2f})"
                    dist_str = f"Dist to target: {actual_distance:.2f}m"
                else:
                    pos_str = "Pos: Unknown"
                    dist_str = ""
                    
                nav.add_frame([
                    f"Step: {step}",
                    f"Target: ({target_x:.2f}, {target_z:.2f})",
                    pos_str,
                    dist_str
                ])
                time.sleep(0.05)

            # Final verification frame
            if reached:
                nav.add_frame([f"TARGET ARRIVAL REACHED!", f"Tolerance: {tolerance}"])
                
            nav.stop_video()
            print(f"Step 3 Complete! Video saved to {video_path}")
        else:
             print("Failed to add agent")
finally:
    nav.close()
