import cv2
import numpy as np
import math

class ObstacleDetector:
    """
    Pure OpenCV obstacle detection and free-space corridor estimator.
    Processes camera frames without AI models, drawing diagnostic HUD overlays.
    """
    def __init__(self, canny_low=50, canny_high=150, min_contour_area=800):
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.min_contour_area = min_contour_area

    def process_frame(self, frame, left_dist=0.0, right_dist=0.0, state_str="IDLE", mode_str="FREE", speed_val=1, goal_info=""):
        """
        Executes the OpenCV vision pipeline:
        1. Resize -> Blur -> Grayscale -> Canny -> Morphology
        2. Contour extraction & filtering -> Bounding boxes
        3. Free corridor estimation
        4. Visualization HUD overlay rendering
        Returns (annotated_frame, obstacle_list, recommended_direction, corridor_info)
        """
        if frame is None:
            return None, [], "CENTER", {"free_width": 0, "confidence": 0.0}

        h, w, _ = frame.shape
        annotated = frame.copy()

        # Step 1: Preprocessing
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        
        # Focus ROI on lower 75% of image (rover's driving corridor)
        roi_mask = np.zeros_like(gray)
        roi_polygon = np.array([[
            (int(w * 0.1), h),
            (int(w * 0.25), int(h * 0.35)),
            (int(w * 0.75), int(h * 0.35)),
            (int(w * 0.9), h)
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_polygon, 255)
        
        # Step 2: Canny Edge Detection & Morphology
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilated = cv2.morphologyEx(masked_edges, cv2.MORPH_CLOSE, kernel)

        # Step 3: Contour Extraction & Bounding Boxes
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        left_sector_blocked = False
        center_sector_blocked = False
        right_sector_blocked = False

        sector_left_bound = w / 3.0
        sector_right_bound = 2.0 * w / 3.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_contour_area:
                x, y, bw, bh = cv2.boundingRect(cnt)
                cx = x + bw // 2
                cy = y + bh // 2

                # Classify position
                if cx < sector_left_bound:
                    pos = "LEFT"
                    left_sector_blocked = True
                elif cx > sector_right_bound:
                    pos = "RIGHT"
                    right_sector_blocked = True
                else:
                    pos = "CENTER"
                    center_sector_blocked = True

                obstacles.append({
                    "box": (x, y, bw, bh),
                    "center": (cx, cy),
                    "area": area,
                    "position": pos
                })

                # Draw obstacle bounding box & center tag
                cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
                cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(annotated, f"OBSTACLE [{pos}]", (x, max(y - 5, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # Step 4: Corridor & Direction Decision
        if not center_sector_blocked and not left_sector_blocked and not right_sector_blocked:
            recommended_dir = "CENTER"
            corridor_color = (0, 255, 0)
        elif not center_sector_blocked:
            recommended_dir = "CENTER"
            corridor_color = (0, 255, 255)
        elif not right_sector_blocked:
            recommended_dir = "RIGHT"
            corridor_color = (0, 165, 255)
        elif not left_sector_blocked:
            recommended_dir = "LEFT"
            corridor_color = (0, 165, 255)
        else:
            recommended_dir = "REVERSE"
            corridor_color = (0, 0, 255)

        # Step 5: Draw Safe Corridor Polygon
        corridor_poly = np.array([
            [int(w * 0.2), h],
            [int(w * 0.35), int(h * 0.4)],
            [int(w * 0.65), int(h * 0.4)],
            [int(w * 0.8), h]
        ], dtype=np.int32)
        
        overlay = annotated.copy()
        cv2.fillPoly(overlay, [corridor_poly], corridor_color)
        cv2.addWeighted(overlay, 0.2, annotated, 0.8, 0, annotated)
        cv2.polylines(annotated, [corridor_poly], True, corridor_color, 2)

        # Step 6: Steering Arrow Overlay
        center_x, center_y = w // 2, h - 40
        arrow_len = 60
        if recommended_dir == "CENTER":
            end_x, end_y = center_x, center_y - arrow_len
        elif recommended_dir == "LEFT":
            end_x, end_y = center_x - arrow_len, center_y - 20
        elif recommended_dir == "RIGHT":
            end_x, end_y = center_x + arrow_len, center_y - 20
        else: # REVERSE
            end_x, end_y = center_x, center_y + 30

        cv2.arrowedLine(annotated, (center_x, center_y), (end_x, end_y), (255, 255, 0), 3, tipLength=0.3)
        cv2.line(annotated, (center_x, h), (center_x, int(h * 0.4)), (255, 255, 255), 1, cv2.LINE_AA)

        # Step 7: HUD Information Dashboard Overlay
        # Header banner
        cv2.rectangle(annotated, (0, 0), (w, 40), (20, 20, 20), -1)
        cv2.putText(annotated, f"DRILLPULSE AUTONOMOUS VIEW | MODE: {mode_str} | STATE: {state_str}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # REC Recording Badge
        cv2.circle(annotated, (w - 30, 20), 8, (0, 0, 255), -1)
        cv2.putText(annotated, "REC", (w - 70, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # Footer dashboard box
        cv2.rectangle(annotated, (0, h - 50), (w, h), (30, 30, 30), -1)
        
        # Sensor overlays
        l_text = f"LEFT: {left_dist:.1f} cm"
        r_text = f"RIGHT: {right_dist:.1f} cm"
        spd_text = f"SPEED: {speed_val}"
        dir_text = f"STEER: {recommended_dir}"

        cv2.putText(annotated, l_text, (15, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if left_dist > 25 else (0, 0, 255), 2)
        cv2.putText(annotated, r_text, (160, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if right_dist > 25 else (0, 0, 255), 2)
        cv2.putText(annotated, spd_text, (310, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)
        cv2.putText(annotated, dir_text, (430, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

        if goal_info:
            cv2.putText(annotated, f"GOAL: {goal_info}", (15, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

        corridor_info = {
            "free_width": w - (max([obs["box"][2] for obs in obstacles], default=0)),
            "left_blocked": left_sector_blocked,
            "center_blocked": center_sector_blocked,
            "right_blocked": right_sector_blocked,
            "confidence": round(1.0 - (len(obstacles) * 0.2), 2)
        }

        return annotated, obstacles, recommended_dir, corridor_info
