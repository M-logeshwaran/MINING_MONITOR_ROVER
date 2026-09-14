import cv2
from drillpulse_autonomous.camera_stream import CameraStream
from drillpulse_autonomous.obstacle_detector import ObstacleDetector
from drillpulse_autonomous.utils import log_colored

class VisionManager:
    """
    Manages camera acquisition, obstacle detection pipeline execution,
    and rendering the 'DRILLPULSE AUTONOMOUS VIEW' OpenCV visualization window.
    """
    def __init__(self, node, topic="/camera/image_raw", width=640, height=480, fps=20, canny_low=50, canny_high=150, simulation=False, webcam_id=0):
        self.node = node
        self.window_name = "DRILLPULSE AUTONOMOUS VIEW"
        
        self.camera_stream = CameraStream(
            node=node,
            topic=topic,
            target_width=width,
            target_height=height,
            fps_target=fps,
            simulation=simulation,
            webcam_id=webcam_id
        )
        
        self.detector = ObstacleDetector(canny_low=canny_low, canny_high=canny_high)
        self.show_window = True

    def process_latest_frame(self, left_dist=0.0, right_dist=0.0, state_str="IDLE", mode_str="FREE", speed_val=1, goal_info=""):
        """
        Retrieves latest camera frame, runs detector pipeline, renders HUD,
        updates visualization window, and returns obstacle analysis results.
        """
        frame, is_connected, current_fps = self.camera_stream.get_latest_frame()
        
        if frame is None:
            return None, [], "CENTER", {"free_width": 0, "confidence": 0.0}, is_connected

        annotated_frame, obstacles, rec_dir, corridor_info = self.detector.process_frame(
            frame=frame,
            left_dist=left_dist,
            right_dist=right_dist,
            state_str=state_str,
            mode_str=mode_str,
            speed_val=speed_val,
            goal_info=goal_info
        )

        # Draw FPS overlay
        if annotated_frame is not None:
            cv2.putText(annotated_frame, f"FPS: {current_fps:.1f}", (10, annotated_frame.shape[0] - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Display Visualization Window
        if self.show_window and annotated_frame is not None:
            try:
                cv2.imshow(self.window_name, annotated_frame)
                cv2.waitKey(1)
            except Exception:
                # Handle headless OpenCV environments without GUI support
                self.show_window = False

        return annotated_frame, obstacles, rec_dir, corridor_info, is_connected

    def destroy_windows(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
