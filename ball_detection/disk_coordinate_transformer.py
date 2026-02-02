import math
import cv2
import numpy as np

class DiskCoordinateTransformer:
    def __init__(self, min_diameter_cm=18.0, max_diameter_cm=19.0):
        self.min_radius_cm = min_diameter_cm / 2
        self.max_radius_cm = max_diameter_cm / 2
        self.disk_radius_cm = (self.min_radius_cm + self.max_radius_cm) / 2
        
        # Fixed disk parameters - sẽ được thiết lập khi biết kích thước frame
        self.center_x = 0
        self.center_y = 0
        self.disk_radius_pixels = 0
        self.pixels_per_cm = None
        
        # Tạo contour đĩa cố định
        self.disk_contour = None
        
    def set_fixed_disk_parameters(self, frame_width, frame_height):
        """
        Thiết lập tham số đĩa cố định dựa trên kích thước frame
        Dựa vào ảnh: đĩa chiếm khoảng 70-75% chiều rộng frame
        """
        # Tâm đĩa ở giữa frame
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2
        
        # Bán kính đĩa dựa trên ảnh (khoảng 80-85% nửa chiều rộng frame)
        # Với frame 640x480, bán kính khoảng 288-310 pixels để khớp với đĩa vàng
        self.disk_radius_pixels = int(min(frame_width, frame_height) * 0.48)  # ~288 pixels cho 640x480
        
        # Tính pixels_per_cm
        self.pixels_per_cm = self.disk_radius_pixels / self.disk_radius_cm
        
        # Tạo contour đĩa cố định
        self._create_fixed_disk_contour()
        
        print(f"Fixed disk parameters set:")
        print(f"  Center: ({self.center_x}, {self.center_y})")
        print(f"  Radius: {self.disk_radius_pixels} pixels")
        print(f"  Pixels per cm: {self.pixels_per_cm:.2f}")
        
    def _create_fixed_disk_contour(self):
        """Tạo contour đĩa cố định dưới dạng đường tròn"""
        if self.disk_radius_pixels > 0:
            # Tạo các điểm trên đường tròn
            angles = np.linspace(0, 2*np.pi, 100)  # 100 điểm
            x_points = self.center_x + self.disk_radius_pixels * np.cos(angles)
            y_points = self.center_y + self.disk_radius_pixels * np.sin(angles)
            
            # Tạo contour
            points = np.column_stack([x_points, y_points])
            self.disk_contour = np.array(points, dtype=np.int32)
        
    def calibrate_with_disk(self, disk_contour=None):
        """
        Hiện tại sử dụng đĩa cố định, không cần contour đầu vào
        Luôn trả về True vì đĩa đã được thiết lập cố định
        """
        if self.pixels_per_cm is None:
            # Nếu chưa thiết lập, sử dụng kích thước mặc định
            self.set_fixed_disk_parameters(640, 480)
        return True
        
    def image_to_cm_coordinates(self, img_x, img_y):
        """Convert image coordinates to cm coordinates relative to disk center"""
        if self.pixels_per_cm is None or self.disk_radius_cm is None:
            return None, None
            
        # Convert to cm relative to disk center
        cm_x = (img_x - self.center_x) / self.pixels_per_cm
        cm_y = (img_y - self.center_y) / self.pixels_per_cm  # Fixed: removed negative sign to flip Y axis
        
        # Apply rotation correction to align with servo coordinate system
        # Rotation angle adjusted for flipped Y axis: from (-8, -4.4) to (9.4, 0)
        rotation_angle = 2.6373  # +151.12 degrees in radians (flipped due to Y axis change)
        cos_angle = math.cos(rotation_angle)
        sin_angle = math.sin(rotation_angle)
        
        # Apply rotation matrix
        rotated_x = cm_x * cos_angle - cm_y * sin_angle
        rotated_y = cm_x * sin_angle + cm_y * cos_angle
        
        # Calculate distance from center in cm
        distance_cm = math.sqrt(rotated_x**2 + rotated_y**2)
        
        # Check if point is outside disk radius
        if distance_cm > self.disk_radius_cm:
            return None, None
            
        return rotated_x, rotated_y