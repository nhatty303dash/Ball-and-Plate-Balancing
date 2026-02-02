import cv2 
import numpy as np
import imutils
import math
from disk_coordinate_transformer import DiskCoordinateTransformer
import logging
import sys
import time
import threading
from queue import Queue
import multiprocessing as mp
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])        
        predicted = self.kf.predict()
        self.kf.correct(measured)
        return predicted

class CoordinatePlotter:
    def __init__(self, queue):
        self.queue = queue
        self.x_data = deque(maxlen=50)  # Store last 50 points
        self.y_data = deque(maxlen=50)  # Store last 50 points
        
        # Setup the plot
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True)
        self.ax.set_title('Ball Position (X,Y)')
        self.ax.set_xlabel('X Position (cm)')
        self.ax.set_ylabel('Y Position (cm)')
        self.line, = self.ax.plot([], [], 'ro-', linewidth=1, markersize=3)
        
    def update(self):
        while True:
            coords = self.queue.get(timeout=0.1)
            if coords is None:  # Exit signal
                break
                
            x, y = coords
            self.x_data.append(x)
            self.y_data.append(y)
            
            self.line.set_data(list(self.x_data), list(self.y_data))
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        
        plt.close()

def plot_process_function(queue):
    plotter = CoordinatePlotter(queue)
    plotter.update()

class BallObjectDetector:
    # Constructor for the class
    def __init__(self, camera_index=0, width=640, height=480):  # Giảm từ 800x800 xuống 640x480 để tăng tốc độ xử lý
        # Initialize multiprocessing components
        self.coord_queue = mp.Queue(maxsize=10)
        self.display_queue = mp.Queue(maxsize=2)
        self.plot_process = None
        self.display_process = None
        self.is_running = mp.Value('b', False)
        
        # Add Kalman filter
        self.kf = KalmanFilter()
        self.predicted_coords = np.zeros((2, 1), np.float32)
        
        # Simple trajectory tracking
        self.trajectory_points = deque(maxlen=20)  # Store last 20 ball centers for shorter trail
        
        # Initialize a list to store the centroids of the ball from previous frames
        self.previous_centroids = []
        
        # Create a counter (dictionary) to track the occurrence of each centroid
        self.centroid_counter = {}
        
        # Frame caching for HSV preview to reduce CPU load
        self.cached_frame = None
        self.cache_timestamp = 0
        self.cache_timeout = 0.033  # Cache frame for 33ms (30 FPS)
        
        # Camera setup - use the specified camera_index
        self.camera_index = int(camera_index)
        # Ensure camera_index is an integer and within valid range
        ###
        # try:
        #     self.camera_index = int(camera_index)
        #     if self.camera_index < 0 or self.camera_index > 10:
        #         raise ValueError(f"Camera index {camera_index} is out of supported range (0-10)")
        # except (ValueError, TypeError):
        #     self.camera_index = 1  # Default to USB camera if error
        # Initialize initial attributes
        ###
        self.cap = None
        self.width = width
        self.height = height
        self.transformer = DiskCoordinateTransformer(min_diameter_cm=18, max_diameter_cm=19)
        # Set up fixed disk parameters based on frame size
        self.transformer.set_fixed_disk_parameters(width, height)
        self.disk_contour = self.transformer.disk_contour
        
        # HSV ranges for different ball colors
        self.ball_colors = {
            'yellow': {
                'lower': np.array([130, 34, 147]),   # Light yellow ball
                'upper': np.array([180, 161, 255]),
                'name': 'Yellow'
            },
            'white': {
                'lower': np.array([60, 5, 148]),     # White ball
                'upper': np.array([136, 175, 255]),
                'name': 'White'
            },
            'red': {
                'lower': np.array([130, 34, 147]),     # Red ball
                'upper': np.array([180, 161, 255]),
                'name': 'Red'
            }
        }
        
        # Current selected ball color
        self.current_ball_color = 'yellow'  # Default to yellow
        self.ball_hsv_lower = self.ball_colors[self.current_ball_color]['lower']
        self.ball_hsv_upper = self.ball_colors[self.current_ball_color]['upper']
        
        # Try different camera connection methods
        # Try DirectShow first
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            # If DirectShow fails, try opening directly
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                # Try with other backends
                backends = [cv2.CAP_ANY, cv2.CAP_V4L2, cv2.CAP_MSMF]
                for backend in backends:
                    self.cap = cv2.VideoCapture(self.camera_index + cv2.CAP_DSHOW, backend)
                    if self.cap.isOpened():
                        break
    
        # If camera still not opened, return with error
        if not self.cap.isOpened():
            self.cap = None
            return
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Set FPS to 30 for stable performance
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Camera max 30 FPS
        
        # Set buffer size to minimum for lowest latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Giữ buffer size = 1 để giảm độ trễ
        
        # Optional: Set exposure for stable FPS (uncomment if needed)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure mode
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Lower exposure for faster FPS
        
        # Verify camera settings
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

    def _open_camera(self, camera_index, width, height):
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            return None
            
        # Set camera properties
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Verify settings
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        return cap

    def _adjust_brightness(self, frame, value):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = np.clip(v + value, 0, 255)  # Ensure pixel values remain in valid range
        hsv = cv2.merge((h, s, v))
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    def _find_disk(self, frame):
        """Find the disk in the frame, focusing only on the circular disk"""
        # Create a copy of the frame for processing
        original = frame.copy()
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply blur to reduce noise (reduce kernel size for speed)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1)
        
        # Estimate disk size based on frame size
        frame_height, frame_width = frame.shape[:2]
        expected_radius_px = min(frame_width, frame_height) * 0.3  # Restore to 0.3
        
        # Use HoughCircles to detect circles
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT, 
            dp=1.5,  # Tăng từ 1.2 lên 1.5 để giảm độ phức tạp tính toán
            minDist=frame_height/2,  # Only find the largest circle
            param1=40,               # Giảm từ 50 xuống 40 để tăng tốc độ
            param2=35,               # Giảm từ 40 xuống 35 để tăng tốc độ
            minRadius=int(expected_radius_px*0.5),  # Restore to 0.5
            maxRadius=int(expected_radius_px*1.8)   # Restore to 1.8
        )
        
        # If a circle is found
        if circles is not None:
            # Convert to int
            circles = np.uint16(np.around(circles))
            
            # Only take the largest circle (first)
            circle = circles[0, 0]
            center = (circle[0], circle[1])
            radius = circle[2]
            
            # Create circular contour
            angles = np.linspace(0, 2*np.pi, 100)
            x_points = center[0] + radius * np.cos(angles)
            y_points = center[1] + radius * np.sin(angles)
            points = np.vstack((x_points, y_points)).T
            disk_contour = np.array(points, dtype=np.int32)
            
            # Check if center is near the frame center
            center_x, center_y = center
            distance_to_center = np.sqrt((center_x - frame_width/2)**2 + (center_y - frame_height/2)**2)
            
            # Debug info (commented to reduce spam)
            # print(f"HoughCircles detected: center=({center_x}, {center_y}), radius={radius}, distance_to_center={distance_to_center:.1f}")
            
            if distance_to_center > frame_width/3:  # If too far from center, may be wrong
                # print(f"Disk center too far from frame center: {distance_to_center:.1f} > {frame_width/3:.1f}")
                return None
            
            # print("Disk detected successfully using HoughCircles")
            return disk_contour

        return None

    def _auto_adjust_lighting(self, frame):
        """
        Advanced lighting adjustment using CLAHE for better ball detection in bright conditions
        """
        # Convert to LAB color space for better lighting processing
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to L channel
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_channel_clahe = clahe.apply(l_channel)
        
        # Merge channels back
        lab_clahe = cv2.merge([l_channel_clahe, a_channel, b_channel])
        
        # Convert back to BGR
        frame_clahe = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)
        
        # Additional contrast and brightness adjustment if needed
        alpha = 1.05  # Slight contrast adjustment
        beta = 5      # Slight brightness adjustment
        
        # Apply formula: output = alpha*input + beta
        adjusted = cv2.convertScaleAbs(frame_clahe, alpha=alpha, beta=beta)
        
        return adjusted

    def _find_contours(self, frame):
        """Find contours in the frame with fixed disk"""
        # Convert to grayscale for ball detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise (optimized for 30 FPS)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)  # Smaller kernel for faster processing
        
        # Use fixed disk contour - always available (no need to reassign each frame)
        # self.disk_contour = self.transformer.disk_contour  # Already set in constructor
        
        # No need to calibrate every frame - already done in constructor
        
        # Convert to HSV for ball detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for white ball
        ball_mask = cv2.inRange(hsv, self.ball_hsv_lower, self.ball_hsv_upper)
        
        # Apply minimal morphological operations for maximum speed
        # Use single smaller kernel for CLOSE only
        kernel = np.ones((3, 3), np.uint8)  # Reduced from (5,5) to (3,3) for speed
        ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_CLOSE, kernel)
        # Skip OPEN operation to save processing time
        
        # Find ball contours
        ball_cnts = cv2.findContours(ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_cnts = imutils.grab_contours(ball_cnts)
        
        return ball_cnts, blurred

    def _find_roundness(self, c):
        perimeter = cv2.arcLength(c, True)
        area = cv2.contourArea(c)
        if perimeter == 0:
            return 0
        return 4 * np.pi * (area / (perimeter ** 2))

    def set_ball_color(self, color_name):
        """Set the ball color to detect - called from GUI"""
        if color_name in self.ball_colors:
            self.current_ball_color = color_name
            self.ball_hsv_lower = self.ball_colors[color_name]['lower']
            self.ball_hsv_upper = self.ball_colors[color_name]['upper']
            return True
        return False

    def get_available_colors(self):
        """Get list of available ball colors for GUI"""
        return list(self.ball_colors.keys())

    def _process_contours(self, cnts):
        centers = []
        contour_data = []

        for c in cnts:
            area = cv2.contourArea(c)
            # Tăng ngưỡng area để loại bỏ các lỗ tròn nhỏ
            if area > 2000:  # Tăng từ 1000 lên 3000 để chỉ bắt bóng lớn
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
                    circularity = self._find_roundness(c)
                    if circularity > 0.6:  # Tăng từ 0.5 lên 0.6 để yêu cầu bóng tròn hơn
                        rect = cv2.minAreaRect(c)
                        box = np.int32(cv2.boxPoints(rect))

                        contour_data.append({
                            'contour': c,
                            'center': (cx, cy),
                            'circularity': circularity,
                            'box': box,
                            'area': area  # Thêm lại area để debug
                        })
                        
        if contour_data:
            # Sort by circularity instead of area for better ball detection
            contour_data = sorted(contour_data, key=lambda x: x['circularity'], reverse=True)[:1]
            centers = [data['center'] for data in contour_data]
        return centers, contour_data

    def _update_centroids(self, centers, threshold=30, persistence=5):
        """Update centroids with adjusted parameters for faster response"""
        stable_positions = []

        for cx, cy in centers:
            found = False
            for i, (prev_cx, prev_cy) in enumerate(self.previous_centroids):
                dist = math.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                if dist < threshold:  # Increased threshold for more stable tracking
                    self.centroid_counter[i] += 1
                    if self.centroid_counter[i] > persistence:  # Reduced persistence for faster response
                        stable_positions.append((prev_cx, prev_cy))
                    self.previous_centroids[i] = (cx, cy)
                    found = True
                    break

            if not found:
                self.previous_centroids.append((cx, cy))
                self.centroid_counter[len(self.previous_centroids) - 1] = 1

        # Clean up old centroids
        self.previous_centroids = [c for i, c in enumerate(self.previous_centroids) 
                                 if self.centroid_counter[i] > 0]
        self.centroid_counter = {i: self.centroid_counter[i] 
                               for i in self.centroid_counter 
                               if self.centroid_counter[i] > 0}

        return stable_positions

    def _sharpen_frame(self, frame):
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        sharp_frame = cv2.filter2D(frame, -1, kernel)
        return sharp_frame

    def _draw_on_frame(self, frame, contour_data, stable_positions):
        if frame is not None:
            display_frame = frame.copy()
        else:
            return None, None

        # Always draw fixed disk contour
        if self.disk_contour is not None:
            # Draw disk contour in dark blue with thickness 2
            cv2.drawContours(display_frame, [self.disk_contour], -1, (255, 0, 0), 2)
            
            # Draw disk center in dark blue (fixed position)
            disk_cx, disk_cy = self.transformer.center_x, self.transformer.center_y
            cv2.circle(display_frame, (disk_cx, disk_cy), 5, (255, 0, 0), -1)
            
            # Draw small cross at disk center
            cv2.line(display_frame, (disk_cx - 7, disk_cy), (disk_cx + 7, disk_cy), (255, 0, 0), 2)
            cv2.line(display_frame, (disk_cx, disk_cy - 7), (disk_cx, disk_cy + 7), (255, 0, 0), 2)
            
            # Display disk info
            cv2.putText(display_frame, f"FIXED DISK - R:{self.transformer.disk_radius_pixels}px", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Draw stable ball positions
        for pos in stable_positions:
            cv2.circle(display_frame, pos, 10, (0, 0, 255), 2)

        # Draw detected ball info and get Kalman predicted coordinates
        predicted_coordinates = None
        for data in contour_data:
            (x, y), radius = cv2.minEnclosingCircle(data['contour'])
            center = (int(x), int(y))
            radius = int(radius)
            
            # Draw enclosing circle
            cv2.circle(display_frame, center, radius, (0, 255, 0), 2)
            
            # Draw ball center
            cx, cy = data['center']
            cv2.circle(display_frame, (cx, cy), 5, (255, 0, 0), -1)
            
            # Add ball center to trajectory
            self.trajectory_points.append((cx, cy))
            
            # Add Kalman prediction
            self.predicted_coords = self.kf.Estimate(cx, cy)
            predicted_x = int(self.predicted_coords[0])
            predicted_y = int(self.predicted_coords[1])
            
            # Store predicted coordinates for return
            predicted_coordinates = (predicted_x, predicted_y)
            
            # Draw Kalman Filter Predicted output with thicker lines
            cv2.circle(display_frame, (predicted_x, predicted_y), 25, [255,255,255], 4, 8)  # Changed color to white
            cv2.line(display_frame, (predicted_x + 20, predicted_y - 20), (predicted_x + 60, predicted_y - 40), [100, 10, 255], 3, 8)  # Increased line length and thickness
            cv2.putText(display_frame, "Predicted", (predicted_x + 60, predicted_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, [50, 200, 250], 2)  # Increased text size and thickness
            
            # Show ball coordinates in disk coordinate system
            cm_x, cm_y = self.transformer.image_to_cm_coordinates(cx, cy)
            if cm_x is not None and cm_y is not None:
                # Show pixel coordinates
                cv2.putText(display_frame, f"Img X={int(cx)}", (cx - 40, cy - 25), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_frame, f"Img Y={int(cy)}", (cx - 40, cy - 10), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                
                # Show cm coordinates
                cv2.putText(display_frame, f"X={cm_x:.1f}cm", (cx - 40, cy + 15), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Y={cm_y:.1f}cm", (cx - 40, cy + 30), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 0, 0), 2)

            # Show ball detection status
            status = "BALL" if data['circularity'] > 0.6 else "NOT BALL"
            color = (0, 255, 0) if status == "BALL" else (0, 0, 255)    
            cv2.putText(display_frame, f"{status} ({data['circularity']:.2f})", 
                      (cx - 40, cy + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Show area for debugging
            cv2.putText(display_frame, f"Area: {int(data['area'])}", 
                      (cx - 40, cy + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw trajectory lines
        if len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                pt1 = self.trajectory_points[i]
                pt2 = self.trajectory_points[i + 1]
                cv2.line(display_frame, pt1, pt2, (0, 255, 255), 2)

        return display_frame, predicted_coordinates

    def _display_process_function(self, queue, is_running):
        """Separate process for displaying frames"""
        while is_running.value:
            combined_display = queue.get(timeout=1.0)
            if combined_display is not None:
                cv2.imshow("Ball Detection", combined_display)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                    is_running.value = False
                    break
        cv2.destroyAllWindows()

    def _processing_function(self):
        """Main processing function"""
        while self.is_running.value:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            frame = cv2.flip(frame, 1)
            height, width = frame.shape[:2]
            
            # Create a black canvas twice the width to show images side by side
            combined_display = np.zeros((height, width * 2, 3), dtype=np.uint8)
            
            # Get contours and binary mask from processing
            cnts, binary_mask = self._find_contours(frame)
            centers, contour_data = self._process_contours(cnts)
            stable_positions = self._update_centroids(centers)
            
            # Draw detections on the original frame
            display_frame, predicted_coordinates = self._draw_on_frame(frame, contour_data, stable_positions)
            
            # Send Kalman predicted coordinates to plotting process if available
            if predicted_coordinates is not None:
                predicted_x, predicted_y = predicted_coordinates
                cm_x, cm_y = self.transformer.image_to_cm_coordinates(predicted_x, predicted_y)
                if cm_x is not None and cm_y is not None:
                    if not self.coord_queue.full():
                        self.coord_queue.put((cm_x, cm_y))
            
            # Convert binary mask to BGR for display
            mask_display = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
            
            # Copy the frames to the combined display
            if display_frame is not None:
                combined_display[:, :width] = display_frame
                combined_display[:, width:] = mask_display
                
                # Add separator line and labels
                cv2.line(combined_display, (width, 0), (width, height), (0, 255, 0), 2)
                cv2.putText(combined_display, "Original", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(combined_display, "Threshold", (width + 10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Put frame in queue for display process
                if not self.display_queue.full():
                    self.display_queue.put(combined_display)

    def detect_objects(self):
        """Start object detection with separate processes"""
        if self.cap is None:
            try:
                self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception as e:
                return
        
        self.is_running.value = True
        
        # Start coordinate plotting process
        self.plot_process = mp.Process(target=plot_process_function, args=(self.coord_queue,))
        self.plot_process.daemon = True
        self.plot_process.start()
        
        # Start display process
        self.display_process = mp.Process(target=self._display_process_function, 
                                        args=(self.display_queue, self.is_running))
        self.display_process.daemon = True
        self.display_process.start()
        
        # Start processing in main process
        try:
            self._processing_function()
        except KeyboardInterrupt:
            pass
        finally:
            self.release()

    def release(self):
        """Clean up resources"""
        self.is_running.value = False
        
        # Send exit signal to plotting process
        if not self.coord_queue.full():
            self.coord_queue.put(None)
        
        # Wait for processes to finish
        if self.plot_process:
            self.plot_process.join(timeout=1.0)
        if self.display_process:
            self.display_process.join(timeout=1.0)
            
        # Release camera
        if self.cap is not None:
            self.cap.release()
        
        # Clear queues
        for q in [self.coord_queue, self.display_queue]:
            while not q.empty():
                q.get_nowait()

    def get_gray_threshold(self):
        """Get binary threshold of current frame using cached frame."""
        current_time = time.time()
        
        # Use cached frame if available and not expired
        if (self.cached_frame is not None and 
            current_time - self.cache_timestamp < self.cache_timeout):
            frame = self.cached_frame
        else:
            # Get new frame and cache it
            ret, frame = self.cap.read()
            if not ret:
                return None
            frame = cv2.flip(frame, 1)
            self.cached_frame = frame.copy()
            self.cache_timestamp = current_time
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Otsu's thresholding
        _, thresh = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresh

    def update_hsv_range(self, lower, upper):
        """Update HSV range from external source (like GUI model)"""
        self.ball_hsv_lower = lower.copy() if hasattr(lower, 'copy') else np.array(lower)
        self.ball_hsv_upper = upper.copy() if hasattr(upper, 'copy') else np.array(upper)

    def get_hsv_preview(self):
        """Return HSV mask and filtered image for preview with frame caching."""
        current_time = time.time()
        
        # Use cached frame if available and not expired
        if (self.cached_frame is not None and 
            current_time - self.cache_timestamp < self.cache_timeout):
            frame = self.cached_frame
        else:
            # Get new frame and cache it
            ret, frame = self.cap.read()
            if not ret:
                return None, None
            frame = cv2.flip(frame, 1)
            self.cached_frame = frame.copy()
            self.cache_timestamp = current_time
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.ball_hsv_lower, self.ball_hsv_upper)
        filtered = cv2.bitwise_and(frame, frame, mask=mask)
        return mask, filtered