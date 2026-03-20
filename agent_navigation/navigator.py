import sys
import time
import json
import cv2
import numpy as np

sys.path.append('/data/code/virtualhome/virtualhome/simulation')
from unity_simulator.comm_unity import UnityCommunication

class AgentNavigator:
    def __init__(self, port='8080', exec_path=None, no_graphics=True, x_display="1"):
        import subprocess
        import os
        
        print(f"Connecting to VirtualHome on port {port}...")
        print("Killing any lingering simulator processes...")
        subprocess.run(['pkill', '-f', 'linux_exec'], stderr=subprocess.DEVNULL)
        time.sleep(2)

        print(f"Launching Unity Simulator manually on DISPLAY :{x_display}...")
        env = os.environ.copy()
        if x_display:
            env['DISPLAY'] = ':' + x_display
            
        self.proc = subprocess.Popen(
            [exec_path, f'-http-port={port}', '-logFile', f'/tmp/Player_{port}.log'],
            env=env,
            start_new_session=True
        )
        print(f"Simulator launched with PID {self.proc.pid}. Waiting for it to initialize...")
        time.sleep(8) # give it time to load scene

        # Use file_name=None to connect to the running instance
        self.comm = UnityCommunication(port=port, file_name=None)
        self.cam_idx = None
        self.video_writer = None

    def reset(self, environment=0):
        print(f"Resetting environment {environment}...")
        return self.comm.reset(environment)

    def add_agent(self, character_resource='Chars/Male1', cam_name='agent_view'):
        print("\n--- Adding Character Camera ---")
        # Standard eye level height is 1.65m. Z offset of 0.2m forwards to avoid head clipping.
        success_cam, msg = self.comm.add_character_camera(position=[0, 1.65, 0.2], rotation=[0, 0, 0], name=cam_name)
        if not success_cam:
            print(f"Failed to add character camera: {msg}")
            # Do not return False if it fails because it already exists? 
            # Usually it returns success.
            
        print("\n--- Adding Character ---")
        success_char = self.comm.add_character(character_resource)
        if not success_char:
            print("Failed to add character")
            return False

        # Static Top view camera index confirmed by user/test
        self.top_cam_idx = 78
        print(f"Using Static Topview Camera index: {self.top_cam_idx}")

        success_names, names_str = self.comm.character_cameras()
        if not success_names:
            print("Failed to get character cameras")
            return False

        names = json.loads(names_str)
        print(f"Available Character Cameras: {names}")
        if cam_name in names:
            relative_idx = names.index(cam_name)
            # Find total cameras to compute absolute index offset
            success_cnt, total_cameras = self.comm.camera_count()
            if success_cnt:
                # Character cameras are appended after static scene cameras
                # Absolute Index = (Total - Character Camera Count) + Relative Index
                self.cam_idx = (total_cameras - len(names)) + relative_idx
                print(f"Mapped Character Camera '{cam_name}' to Absolute Index: {self.cam_idx}")
                return True
            else:
                print("Failed to get total camera count for offset mapping")
                return False
        else:
            print(f"Camera '{cam_name}' not found")
            return False

    def move_character(self, pos):
        # Ensure pos is a list
        success = self.comm.move_character(0, list(pos))
        if success:
            # Empty script tick to commit physics in simulator
            self.comm.render_script([])
            return True
        return False

    def get_position(self):
        success, graph = self.comm.environment_graph()
        if not success:
            return None
        
        for node in graph['nodes']:
            if node.get('class_name') == 'character':
                transform = node.get('obj_transform')
                if transform:
                    pos = transform.get('position')
                    rot = transform.get('rotation')
                    return pos, rot
        return None

    def start_video(self, filename, dual_view=False, fps=10):
        self.dual_view = dual_view
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # Double width for side-by-side view
        width = 1280 if dual_view else 640
        height = 480
        self.video_writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
        print(f"Started video recording: {filename} (Dual: {dual_view})")

    def add_frame(self, text_overlays=[]):
        if self.video_writer is None:
            return False

        cams = []
        if hasattr(self, 'top_cam_idx') and self.top_cam_idx is not None:
             cams.append(self.top_cam_idx)
        if hasattr(self, 'dual_view') and self.dual_view and self.cam_idx is not None:
             cams.append(self.cam_idx)
             
        if not cams:
             print("No active cameras for recording")
             return False

        success_rgb, rgb = self.comm.camera_image(cams)
        if success_rgb and len(rgb) > 0:
            if hasattr(self, 'dual_view') and self.dual_view and len(rgb) == 2:
                 # rgb[0] = top, rgb[1] = person
                 frame = np.hstack((rgb[0], rgb[1]))
            else:
                 frame = rgb[0]
                 
            # Convert to BGR for OpenCV
            frame_bgr = frame # cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Draw HUD
            y_offset = 30
            for text in text_overlays:
                cv2.putText(frame_bgr, str(text), (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                y_offset += 20

            self.video_writer.write(frame_bgr)
            return True
        return False

    def stop_video(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            print("Stopped video recording.")

    def close(self):
        self.stop_video()
        self.comm.close()
        # Kill the child subprocess manually launched
        if hasattr(self, 'proc') and self.proc is not None:
             print("Killing simulator subprocess...")
             self.proc.kill()
             self.proc = None
        print("Connection closed.")
