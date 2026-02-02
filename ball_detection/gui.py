import sys
import cv2
import numpy as np
import math
import time
from collections import deque
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSlider, QSizePolicy, QGroupBox, QDoubleSpinBox, QMessageBox, QFrame, QLineEdit, 
    QComboBox, QSpinBox, QTextEdit, QCheckBox, QScrollArea, QTabWidget
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QFont

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from circular_object import BallObjectDetector
from serial_communication import SerialCommunication

# --- VIEW ------------------------------------------------------------------
class BallValuesWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Ball Position Values')
        self.setStyleSheet("background-color: white; color: black;")
        
        # Initialize data storage with more points for longer time window
        self.max_points = 2000  # Tăng số điểm để lưu được 20 giây
        self.x_values = deque(maxlen=self.max_points)
        self.y_values = deque(maxlen=self.max_points)
        self.time_points = deque(maxlen=self.max_points)
        self.start_time = None
        self.last_update = 0
        self.update_interval = 50
        self.time_window = 20  # Hiển thị 20 giây dữ liệu
        
        # Setup matplotlib with optimization
        matplotlib.use('Qt5Agg')
        plt.style.use('fast')
        plt.rcParams['animation.html'] = 'none'
        
        self._build_ui()
        
        # Use QTimer with increased frequency
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(self.update_interval)
        
        # Double buffering for plot data
        self._buffer_time = []
        self._buffer_x = []
        self._buffer_y = []
        
        # Cache for background
        self._backgrounds = []

    def _build_ui(self):
        layout = QVBoxLayout(self)
        
        # Optimize figure creation
        plt.rcParams['figure.dpi'] = 100
        plt.rcParams['figure.autolayout'] = True
        plt.rcParams.update({
            'text.color': 'black',
            'axes.labelcolor': 'black',
            'xtick.color': 'black',
            'ytick.color': 'black',
            'figure.facecolor': 'white',
            'axes.facecolor': 'white',
            'axes.grid': False,
        })
        
        # Create figure with optimized parameters and larger size
        self.figure, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6), 
                                                        facecolor='white',
                                                        constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setParent(self)
        layout.addWidget(self.canvas)
        
        # Pre-configure axes for better performance
        for ax, title in [
            (self.ax1, 'Vị trí bi theo phương X'),
            (self.ax2, 'Vị trí bi theo phương Y')
        ]:
            ax.set_title(title, fontsize=10, color='black')
            ax.set_xlabel('Time (s)', fontsize=8, color='black')
            ax.set_ylabel('Position (mm)', fontsize=8, color='black')
            ax.set_ylim(-95, 95)
            
            # Set major ticks every 10mm with small font size
            major_ticks = np.arange(-90, 91, 10)
            ax.set_yticks(major_ticks)
            ax.set_yticklabels(major_ticks, fontsize=7)
            
            # Vẽ lưới chính mỗi 10mm
            ax.grid(True, which='major', linestyle='-', alpha=0.2)
            
            # Vẽ vạch chia phụ mỗi 5mm nhưng không hiển thị số
            minor_ticks = np.arange(-95, 96, 5)
            ax.set_yticks(minor_ticks, minor=True)
            ax.tick_params(axis='y', which='major', length=6)
            ax.tick_params(axis='y', which='minor', length=3)
            ax.set_yticklabels([], minor=True)
            
            ax.set_facecolor('white')
            
            # Create setpoint line (y=0)
            ax.axhline(y=0, color='red', linestyle='-', linewidth=1)
            
            # Create empty line objects for updating
            line, = ax.plot([], [], '-', linewidth=0.8, color='blue', alpha=0.6)
            
            # Store line objects for updates
            if ax == self.ax1:
                self.line_x = line
            else:
                self.line_y = line
                
            # Initialize time axis from 0 to 20 seconds
            ax.set_xlim(0, self.time_window)
            
            # Set major ticks every 5 seconds
            major_ticks = np.arange(0, self.time_window + 1, 5)
            ax.set_xticks(major_ticks)
            ax.set_xticklabels([f"{t:.0f}" for t in major_ticks], fontsize=7)
            
            # Set minor ticks every 1 second
            minor_ticks = np.arange(0, self.time_window + 1, 1)
            ax.set_xticks(minor_ticks, minor=True)
            ax.tick_params(axis='x', which='major', length=6)
            ax.tick_params(axis='x', which='minor', length=3)
            
            # Only show grid for major ticks (5s intervals)
            ax.grid(True, which='major', axis='x', linestyle='-', alpha=0.2)
            ax.grid(False, which='minor', axis='x')
        
        # Cache the background for blitting
        self.canvas.draw()
        self._backgrounds = [self.figure.canvas.copy_from_bbox(ax.bbox) for ax in [self.ax1, self.ax2]]

    def _update_plots(self):
        current_time = time.time()
        if current_time - self.last_update < self.update_interval / 1000:
            return
            
        # Update main data from buffer
        if self._buffer_time:
            self.time_points.extend(self._buffer_time)
            self.x_values.extend(self._buffer_x)
            self.y_values.extend(self._buffer_y)
            
            # Clear buffers
            self._buffer_time.clear()
            self._buffer_x.clear()
            self._buffer_y.clear()
        else:
            return  # No new data to plot
            
        # Convert deques to lists for plotting
        times = list(self.time_points)
        x_vals = list(self.x_values)
        y_vals = list(self.y_values)
        
        # Restore backgrounds
        for ax, background in zip([self.ax1, self.ax2], self._backgrounds):
            self.figure.canvas.restore_region(background)
        
        # Update line data
        self.line_x.set_data(times, x_vals)
        self.line_y.set_data(times, y_vals)
        
        # Update x-axis limits to show fixed time window
        if times:
            current_time = times[-1]
            for ax in [self.ax1, self.ax2]:
                # Calculate start time to maintain fixed window
                start_time = max(0, current_time - self.time_window)
                end_time = max(self.time_window, current_time)
                ax.set_xlim(start_time, end_time)
                
                # Set major ticks every 5 seconds
                major_ticks = np.arange(
                    np.floor(start_time / 5) * 5,
                    end_time + 5,
                    5
                )
                ax.set_xticks(major_ticks)
                ax.set_xticklabels([f"{t:.0f}" for t in major_ticks], fontsize=7)
                
                # Set minor ticks every 1 second
                minor_ticks = np.arange(
                    np.floor(start_time),
                    end_time + 1,
                    1
                )
                ax.set_xticks(minor_ticks, minor=True)
                ax.tick_params(axis='x', which='major', length=6)
                ax.tick_params(axis='x', which='minor', length=3)
                
                # Only show grid for major ticks (5s intervals)
                ax.grid(True, which='major', axis='x', linestyle='-', alpha=0.2)
                ax.grid(False, which='minor', axis='x')
        
        # Draw only the updated artists
        self.ax1.draw_artist(self.line_x)
        self.ax2.draw_artist(self.line_y)
        
        # Update the display
        self.canvas.blit(self.ax1.bbox)
        self.canvas.blit(self.ax2.bbox)
        
        self.last_update = current_time

    def showEvent(self, event):
        super().showEvent(event)
        # Re-cache background when window is shown
        self.canvas.draw()
        self._backgrounds = [self.figure.canvas.copy_from_bbox(ax.bbox) for ax in [self.ax1, self.ax2]]
        self.timer.start(self.update_interval)

    def hideEvent(self, event):
        super().hideEvent(event)
        self.timer.stop()

    def closeEvent(self, event):
        self.timer.stop()
        plt.close(self.figure)
        super().closeEvent(event)

    def update_data(self, x, y):
        """Update plot data with new coordinates"""
        if self.start_time is None:
            self.start_time = datetime.now()
        
        # Convert from cm to mm and add to buffer
        t = (datetime.now() - self.start_time).total_seconds()
        self._buffer_time.append(t)
        self._buffer_x.append(x * 10)  # Convert cm to mm
        self._buffer_y.append(y * 10)  # Convert cm to mm


class HSVAdjustmentWindow(QWidget):
    def __init__(self, ball_detector):
        super().__init__()
        self.ball_detector = ball_detector
        self.setWindowTitle('HSV & Gray Threshold Preview')
        self.setStyleSheet("background-color: white; color: black;")
        self._build_ui()
        
        # Reduce timer frequency to 300ms (3.3 FPS) instead of 100ms (10 FPS) to save CPU
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_previews)
        self.timer.start(300)  # Increased from 100ms to 300ms
        
        # Flag to track if window is visible
        self.is_visible = False

    def showEvent(self, event):
        """Called when window becomes visible"""
        super().showEvent(event)
        self.is_visible = True
        # Start timer when window is shown
        if not self.timer.isActive():
            self.timer.start(300)

    def hideEvent(self, event):
        """Called when window becomes hidden"""
        super().hideEvent(event)
        self.is_visible = False
        # Stop timer when window is hidden to save CPU
        self.timer.stop()

    def _build_ui(self):
        main = QVBoxLayout(self)

        # hai màn hình preview
        hl = QHBoxLayout()
        self.gray_label = QLabel()
        self.hsv_mask_label = QLabel()
        for lbl in (self.gray_label, self.hsv_mask_label):
            lbl.setFixedSize(200,200)
            lbl.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
            lbl.setStyleSheet("color: black;")
            hl.addWidget(lbl)
        main.addLayout(hl)

        # sliders HSV lower
        self._make_sliders(main, "Lower HSV", self.ball_detector.ball_hsv_lower, True)
        # sliders HSV upper
        self._make_sliders(main, "Upper HSV", self.ball_detector.ball_hsv_upper, False)

    def _make_sliders(self, layout, title, arr, is_lower):
        layout.addWidget(QLabel(title, styleSheet="color: black; font-weight: bold;"))
        for i, comp in enumerate(('H','S','V')):
            box = QHBoxLayout()
            label = QLabel(comp)
            label.setStyleSheet("color: black;")
            box.addWidget(label)
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(0, 180 if comp=='H' else 255)
            s.setValue(int(arr[i]))
            v = QLabel(str(int(arr[i])))
            v.setStyleSheet("color: black;")
            s.valueChanged.connect(
                lambda val, idx=i, lbl=v, low=is_lower: self._on_hsv_slider(val, idx, lbl, low)
            )
            box.addWidget(s)
            box.addWidget(v)
            layout.addLayout(box)

    def _on_hsv_slider(self, val, idx, label, is_lower):
        label.setText(str(val))
        if is_lower:
            self.ball_detector.ball_hsv_lower[idx] = val
        else:
            self.ball_detector.ball_hsv_upper[idx] = val

    def _refresh_previews(self):
        # Only refresh if window is visible to save CPU
        if not self.is_visible or not self.isVisible():
            return
            
        # Gray threshold preview
        thresh = self.ball_detector.get_gray_threshold()
        if thresh is not None:
            img = QImage(thresh.data, thresh.shape[1], thresh.shape[0], thresh.strides[0], QImage.Format.Format_Grayscale8)
            self.gray_label.setPixmap(QPixmap.fromImage(img).scaled(
                self.gray_label.size(), Qt.AspectRatioMode.KeepAspectRatio))

        # HSV mask preview
        mask, filt = self.ball_detector.get_hsv_preview()
        if mask is not None:
            img2 = QImage(mask.data, mask.shape[1], mask.shape[0], mask.strides[0], QImage.Format.Format_Grayscale8)
            self.hsv_mask_label.setPixmap(QPixmap.fromImage(img2).scaled(
                self.hsv_mask_label.size(), Qt.AspectRatioMode.KeepAspectRatio))

    def closeEvent(self, event):
        """Called when window is closed"""
        # Stop timer when window is closed to save CPU
        if self.timer.isActive():
            self.timer.stop()
        self.is_visible = False
        super().closeEvent(event)





class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Ball Detection GUI')
        
        # Initialize frame counter and FPS calculation variables
        self.frame_count = 0
        self.total_frames = 0  # Tổng số frame đã xử lý
        self.fps = 0
        self.last_fps_update = datetime.now()
        self.fps_update_interval = 0.5  # Update FPS every 0.5 second
        self.frame_skip_counter = 0
        self.frame_skip_rate = 1  # Process all frames for maximum responsiveness
        
        # Initialize serial communication
        self.serial_comm = SerialCommunication()
        
        # Khởi tạo các biến trạng thái
        self.camera_running = False
        self.current_camera_index = 0  # Camera Webcam
        self.ball_detector = None
        self.hsv_window = None
        self.values_window = None
        self.current_ball_pos = (0, 0)
        
        # Giá trị mặc định cho độ phân giải camera
        self.current_width = 640  # Giảm từ 800 xuống 640
        self.current_height = 480  # Giảm từ 800 xuống 480 để tăng tốc độ
        self.current_fps = 30  # Camera tối đa 30 FPS
        
        # Thêm biến để theo dõi thời gian gửi
        self.last_send_time = 0
        self.send_interval = 0.033  # Optimized to 30Hz (33ms) - balanced for ball balancing system
        
        # Khởi tạo giao diện người dùng
        self.setup_ui()
        
        # Create timer for video update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer_interval = 30  # Reduced to 30ms for better 30fps performance
        
        # Cập nhật hiển thị
        self.update_resolution_display()
        
        # Thử kết nối với camera ban đầu sau khi giao diện đã được tạo
        self.try_connect_camera(0)  # Mặc định kết nối Webcam

    def setup_ui(self):
        # Set background color
        self.setStyleSheet("background-color: white;")
        
        # Create central widget with horizontal layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left side control panel
        left_panel = QWidget()
        left_panel.setFixedWidth(250)  # Reduced from 300 to 250
        left_panel.setStyleSheet("background-color: #f0f0f0; border-radius: 10px;")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)  # Reduced spacing
        left_layout.setContentsMargins(8, 15, 8, 15)  # Adjusted margins
        
        # Controls label (centered)
        controls_label = QLabel("Controls")
        controls_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #444;")
        controls_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_layout.addWidget(controls_label)
        
        # Camera selection group
        camera_group = QGroupBox()
        camera_group.setStyleSheet("QGroupBox { border: none; }")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.setSpacing(5)
        
        # Camera selection
        camera_selection_layout = QHBoxLayout()
        camera_label = QLabel("Camera:")
        camera_label.setStyleSheet("font-weight: bold; color: #000000;")
        self.camera_combo = QComboBox()
        self.camera_combo.setStyleSheet("""
            QComboBox {
                background-color: #ffffff;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                color: #000000;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: #000000;
                background-color: #ffffff;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: #000000;
                background-color: #e8f8e8;
                font-weight: bold;
            }
            QComboBox QAbstractItemView {
                color: #000000;
                background-color: #ffffff;
                selection-background-color: #e8f8e8;
                selection-color: #000000;
            }
        """)
        self.camera_combo.addItems(["Camera 0 (Webcam)", "Camera 1 (USB)", "Camera 2", "Camera 3"])
        self.camera_combo.setCurrentIndex(0)  # Mặc định chọn Webcam
        camera_selection_layout.addWidget(camera_label)
        camera_selection_layout.addWidget(self.camera_combo)
        camera_layout.addLayout(camera_selection_layout)
        
        # Camera control buttons
        button_layout = QHBoxLayout()
        button_layout.setSpacing(5)
        
        self.start_camera_button = QPushButton('Start Camera')
        self.start_camera_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60; 
                color: white; 
                border-radius: 3px; 
                padding: 5px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #2ecc71; }
            QPushButton:disabled { background-color: #95a5a6; }
        """)
        
        self.stop_camera_button = QPushButton('Stop Camera')
        self.stop_camera_button.setStyleSheet("""
            QPushButton {
                background-color: #c0392b; 
                color: white; 
                border-radius: 3px; 
                padding: 5px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #e74c3c; }
            QPushButton:disabled { background-color: #95a5a6; }
        """)
        
        button_layout.addWidget(self.start_camera_button)
        button_layout.addWidget(self.stop_camera_button)
        camera_layout.addLayout(button_layout)
        left_layout.addWidget(camera_group)
        
        # Camera Parameters group
        camera_params_group = QGroupBox("Camera Parameters")
        camera_params_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        params_layout = QVBoxLayout(camera_params_group)
        params_layout.setSpacing(5)
        
        self.resolution_label = QLabel("Resolution: --x--")
        self.fps_label = QLabel("FPS: --")
        self.frame_count_label = QLabel("Frames: 0")
        
        for label in [self.resolution_label, self.fps_label, self.frame_count_label]:
            label.setStyleSheet("""
                color: black;
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 3px;
                padding: 3px;
                margin: 2px;
            """)
            params_layout.addWidget(label)
        
        left_layout.addWidget(camera_params_group)
        
        # Control buttons group
        control_buttons_group = QGroupBox()
        control_buttons_group.setStyleSheet("QGroupBox { border: none; }")
        buttons_layout = QVBoxLayout(control_buttons_group)
        buttons_layout.setSpacing(0)  # Đặt về 0 để tự quản lý spacing
        
        # Create control buttons with consistent styling
        button_configs = [
            ('Adjust HSV', '#3498db', '#2980b9'),
            ('Ball Values', '#2ecc71', '#27ae60')
        ]
        
        for text, bg_color, hover_color in button_configs:
            btn = QPushButton(text)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {bg_color};
                    color: white;
                    border-radius: 5px;
                    padding: 12px 8px;
                    font-weight: bold;
                    font-size: 12px;
                    min-height: 35px;
                    text-align: center;
                }}
                QPushButton:hover {{
                    background-color: {hover_color};
                    transform: translateY(-1px);
                }}
                QPushButton:pressed {{
                    transform: translateY(1px);
                }}
            """)
            btn.setMinimumHeight(40)  # Đảm bảo chiều cao tối thiểu
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            buttons_layout.addWidget(btn)
            
            # Thêm khoảng cách giống như giữa Connect button và status label
            buttons_layout.addSpacing(8)
            
            # Connect button signals
            if text == 'Adjust HSV':
                btn.clicked.connect(self.show_hsv_adjustment)
            elif text == 'Ball Values':
                btn.clicked.connect(self.show_ball_values)
        
        left_layout.addWidget(control_buttons_group)
        
        # Mode Selection group
        mode_group = QGroupBox("Mode Selection")
        mode_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        mode_layout = QVBoxLayout(mode_group)
        mode_layout.setSpacing(5)
        
        mode_selection_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        mode_label.setStyleSheet("font-weight: bold; color: #000000;")
        self.mode_combo = QComboBox()
        self.mode_combo.setStyleSheet("""
            QComboBox {
                background-color: #ffffff;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                color: #000000;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: #000000;
                background-color: #ffffff;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: #000000;
                background-color: #e8f8e8;
                font-weight: bold;
            }
            QComboBox QAbstractItemView {
                color: #000000;
                background-color: #ffffff;
                selection-background-color: #e8f8e8;
                selection-color: #000000;
            }
        """)
        self.mode_combo.addItems(["Balance", "Circle", "Square"])
        self.mode_combo.setCurrentIndex(0)  # Mặc định chọn Balance
        mode_selection_layout.addWidget(mode_label)
        mode_selection_layout.addWidget(self.mode_combo)
        mode_layout.addLayout(mode_selection_layout)
        
        left_layout.addWidget(mode_group)
        
        # Ball Position group
        position_group = QGroupBox("Ball Position")
        position_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        position_layout = QVBoxLayout(position_group)
        position_layout.setSpacing(5)
        
        # Ball Color selection
        color_layout = QHBoxLayout()
        color_label = QLabel("Ball Color:")
        color_label.setStyleSheet("font-weight: bold; color: #8e44ad; font-size: 12px;")
        
        self.ball_color_combo = QComboBox()
        self.ball_color_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: black;
                background-color: white;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: black;
                background-color: #e8f8e8;
            }
            QComboBox QAbstractItemView {
                color: black;
                background-color: white;
                selection-background-color: #e8f8e8;
                selection-color: black;
            }
        """)
        self.ball_color_combo.addItems(["Yellow", "White", "Red"])
        self.ball_color_combo.setCurrentText("Red")  # Default to red
        self.ball_color_combo.currentTextChanged.connect(self.change_ball_color)
        
        color_layout.addWidget(color_label)
        color_layout.addWidget(self.ball_color_combo)
        position_layout.addLayout(color_layout)
        
        # X and Y coordinates
        for coord, color in [('X:', '#e74c3c'), ('Y:', '#3498db')]:
            coord_layout = QHBoxLayout()
            label = QLabel(coord)
            label.setStyleSheet(f"font-weight: bold; color: {color}; font-size: 14px;")
            
            value_edit = QLineEdit()
            value_edit.setReadOnly(True)
            value_edit.setStyleSheet("""
                background-color: white;
                color: black;
                padding: 5px;
                border: 1px solid #ddd;
                border-radius: 3px;
                font-weight: bold;
            """)
            value_edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value_edit.setText("0.0 cm")
            
            if coord == 'X:':
                self.x_value_edit = value_edit
            else:
                self.y_value_edit = value_edit
                
            coord_layout.addWidget(label)
            coord_layout.addWidget(value_edit)
            position_layout.addLayout(coord_layout)
        
        left_layout.addWidget(position_group)
        
        # Status group
        status_group = QGroupBox("Status")
        status_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        status_layout = QVBoxLayout(status_group)
        self.status_text = QLabel("Ready")
        self.status_text.setStyleSheet("""
            color: #27ae60;
            background-color: #e8f8e8;
            padding: 5px;
            border-radius: 3px;
            font-weight: bold;
            text-align: center;
        """)
        self.status_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_layout.addWidget(self.status_text)
        
        left_layout.addWidget(status_group)
        
        # Serial Connection group
        serial_group = QGroupBox("Serial Connection")
        serial_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        serial_layout = QVBoxLayout()
        
        # COM Port selection
        port_layout = QHBoxLayout()
        port_label = QLabel("COM Port:")
        port_label.setStyleSheet("color: black;")
        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: black;
                background-color: white;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: black;
                background-color: #e8f8e8;
            }
            QComboBox QAbstractItemView {
                color: black;
                background-color: white;
                selection-background-color: #e8f8e8;
                selection-color: black;
            }
        """)
        # Add available COM ports
        self.port_combo.addItems(self.serial_comm.get_available_ports())
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)
        
        # Baud Rate selection
        baud_layout = QHBoxLayout()
        baud_label = QLabel("Baud Rate:")
        baud_label.setStyleSheet("color: black;")
        self.baud_combo = QComboBox()
        self.baud_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: black;
                background-color: white;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: black;
                background-color: #e8f8e8;
            }
            QComboBox QAbstractItemView {
                color: black;
                background-color: white;
                selection-background-color: #e8f8e8;
                selection-color: black;
            }
        """)
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baud_combo.setCurrentText('115200')  # Default baud rate
        baud_layout.addWidget(baud_label)
        baud_layout.addWidget(self.baud_combo)
        
        # Connect button
        self.connect_button = QPushButton('Connect')
        self.connect_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border-radius: 5px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
        """)
        
        # Add all to serial layout
        serial_layout.addLayout(port_layout)
        serial_layout.addLayout(baud_layout)
        serial_layout.addWidget(self.connect_button)
        
        serial_group.setLayout(serial_layout)
        left_layout.addWidget(serial_group)
        
        # Add stretch at the end to push everything up
        left_layout.addStretch()
        
        # Right side - video display
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(2, 2, 2, 2)  # Reduced margins
        
        # Title - made more compact
        title_label = QLabel("BALL AND PLATE BALANCING SYSTEM")
        title_label.setStyleSheet("""
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 20px; 
            font-weight: 700;
            color: #FF0000;
            padding: 8px;
            background-color: white;
            border-radius: 3px;
            border: 1px solid #bdc3c7;
            margin-bottom: 2px;
            letter-spacing: 1px;
            text-rendering: optimizeLegibility;
            -webkit-font-smoothing: antialiased;
        """)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setMaximumHeight(45)  # Increased height slightly to accommodate new font size
        right_layout.addWidget(title_label)
        
        # Image display
        self.image_label = QLabel()
        self.image_label.setStyleSheet("""
            border: 2px solid #bdc3c7;
            background-color: #f5f6fa;
            border-radius: 5px;
        """)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setFixedSize(800, 800)  # Đặt kích thước vuông để hiển thị đầy đủ
        
        # Center the image label
        image_container = QWidget()
        image_container_layout = QVBoxLayout(image_container)
        image_container_layout.setContentsMargins(0, 0, 0, 0)  # Remove container margins
        image_container_layout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignCenter)
        right_layout.addWidget(image_container)
        
        # Main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, 1)  # Give more space to right panel
        
        # Window settings
        self.setMinimumSize(1200, 900)  # Tăng chiều cao để hiển thị đầy đủ ảnh vuông
        self.center_window()

        # Connect all signals after UI is fully created
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        self.start_camera_button.clicked.connect(self.start_camera)
        self.stop_camera_button.clicked.connect(self.stop_camera)
        self.connect_button.clicked.connect(self.toggle_serial_connection)

    def center_window(self):
        """Center the window on the screen"""
        window_geometry = self.frameGeometry()
        screen_center = QApplication.primaryScreen().geometry().center()
        window_geometry.moveCenter(screen_center)
        self.move(window_geometry.topLeft())

    def update_status(self, message, is_error=False):
        """Update status message with color based on type"""
        if is_error:
            self.status_text.setStyleSheet("color: #c0392b; background-color: #fadbd8; padding: 5px; border-radius: 3px;")
        else:
            self.status_text.setStyleSheet("color: #27ae60; background-color: #e8f8e8; padding: 5px; border-radius: 3px;")
        self.status_text.setText(message)

    def show_hsv_adjustment(self):
        if self.hsv_window is None:
            self.hsv_window = HSVAdjustmentWindow(self.ball_detector)
        self.hsv_window.show()
        self.update_status("HSV adjustment window opened")

    def show_ball_values(self):
        if self.values_window is None:
            self.values_window = BallValuesWindow()
        self.values_window.show()
        self.update_status("Ball values graph opened")



    def change_ball_color(self, color_text):
        """Handle ball color selection change"""
        if self.ball_detector is None:
            return
            
        # Map display text to internal color names
        color_map = {
            "Yellow": "yellow",
            "White": "white", 
            "Red": "red"
        }
        
        if color_text in color_map:
            internal_color = color_map[color_text]
            # Set the ball color in the detector
            if hasattr(self.ball_detector, 'ball_colors'):
                self.ball_detector.current_ball_color = internal_color
                self.ball_detector.ball_hsv_lower = self.ball_detector.ball_colors[internal_color]['lower']
                self.ball_detector.ball_hsv_upper = self.ball_detector.ball_colors[internal_color]['upper']
                self.update_status(f"Ball color changed to: {color_text}")
            else:
                self.update_status("Ball detector not initialized", True)

    def change_camera(self):
        """Change the camera based on selection"""
        # Get selected camera index (0-3)
        selected_index = self.camera_combo.currentIndex()
        
        # Stop the current camera first
        self.stop_camera()
        
        # Try to connect to selected camera
        try:
            # Create new detector with selected camera
            self.ball_detector = BallObjectDetector(camera_index=selected_index)
            
            if not self.ball_detector.cap or not self.ball_detector.cap.isOpened():
                raise Exception(f"Could not open Camera {selected_index}")
            
            # Update camera index
            self.current_camera_index = selected_index
            
            # Get camera properties
            self.current_width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.current_height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.current_fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
            
            # Update display
            self.update_resolution_display()
            
            # Start camera
            self.camera_running = True
            self.timer.start(self.timer_interval)
            
            # Update button states
            self.start_camera_button.setEnabled(False)
            self.stop_camera_button.setEnabled(True)
            
            # Update status
            self.update_status(f"Changed to Camera {selected_index}")
            
        except Exception as e:
            self.camera_running = False
            if self.ball_detector:
                self.ball_detector.release()
            self.ball_detector = None
            
            # Update status
            self.update_status(f"Could not connect to Camera {selected_index}", True)
            self.show_error_image(f"COULD NOT CONNECT TO CAMERA {selected_index}")
            
            # Reset button states
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)

    def try_connect_camera(self, camera_index):
        """Thử kết nối đến camera với chỉ số đã cho"""
        # Cập nhật trạng thái
        self.update_status(f"Connecting to Camera {camera_index}...")
        
        try:
            # Tạo đối tượng detector mới với camera mới
            self.ball_detector = BallObjectDetector(camera_index=camera_index)
            self.current_camera_index = camera_index
            
            # Lấy thông tin độ phân giải
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap:
                self.current_width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                self.current_height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                self.current_fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
                self.update_resolution_display()
            
            # Cập nhật trạng thái camera
            self.camera_running = True
            self.update_status(f"Connected to Camera {camera_index}")
            
            # Cập nhật trạng thái nút
            self.start_camera_button.setEnabled(False)
            self.stop_camera_button.setEnabled(True)
            
            # Bắt đầu timer
            self.timer.start(self.timer_interval)
            
        except Exception as e:
            self.camera_running = False
            self.ball_detector = None  # Đảm bảo đối tượng detector là None nếu có lỗi
            self.update_status(f"Error connecting to camera {camera_index}", True)
            
            # Hiển thị ảnh thông báo lỗi
            self.show_error_image(f"COULD NOT CONNECT TO CAMERA {camera_index}")
            
            # Cập nhật trạng thái nút
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)

    def show_error_image(self, message):
        """Hiển thị ảnh thông báo lỗi"""
        if hasattr(self, 'image_label'):
            # Tạo hình ảnh thông báo
            error_img = np.ones((480, 640, 3), dtype=np.uint8) * 240
            
            # Vẽ chữ chính giữa với font to và rõ ràng
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale_main = 1.2
            thickness_main = 3
            text_color = (0, 0, 255)  # Màu đỏ
            
            # Map English messages to Vietnamese
            message_map = {
                "COULD NOT CONNECT TO CAMERA": "COULD NOT CONNECT TO CAMERA",
                "CAMERA DISCONNECTED": "CAMERA DISCONNECTED",
                "COULD NOT CONNECT TO CAMERA 0": "COULD NOT CONNECT TO CAMERA 0",
                "COULD NOT CONNECT TO CAMERA 1": "COULD NOT CONNECT TO CAMERA 1",
                "COULD NOT CONNECT TO CAMERA 2": "COULD NOT CONNECT TO CAMERA 2",
                "COULD NOT CONNECT TO CAMERA 3": "COULD NOT CONNECT TO CAMERA 3"
            }
            
            # Convert message if it exists in the map
            display_message = message_map.get(message, message)
            
            # Căn giữa dòng text chính
            text_size = cv2.getTextSize(display_message, font, font_scale_main, thickness_main)[0]
            text_x = (640 - text_size[0]) // 2
            text_y = 220
            
            # Vẽ text chính
            cv2.putText(error_img, display_message, (text_x, text_y), 
                       font, font_scale_main, text_color, thickness_main, cv2.LINE_AA)
            
            # Vẽ dòng text phụ
            sub_message = "Please check camera connection"  # Changed to Telex
            font_scale_sub = 0.8
            thickness_sub = 2
            
            # Căn giữa dòng text phụ
            sub_text_size = cv2.getTextSize(sub_message, font, font_scale_sub, thickness_sub)[0]
            sub_text_x = (640 - sub_text_size[0]) // 2
            sub_text_y = 280
            
            cv2.putText(error_img, sub_message, (sub_text_x, sub_text_y), 
                       font, font_scale_sub, text_color, thickness_sub, cv2.LINE_AA)
            
            height, width = error_img.shape[:2]
            bytes_per_line = 3 * width
            q_image = QImage(error_img.data, width, height, bytes_per_line, 
                            QImage.Format.Format_RGB888).rgbSwapped()
            
            self.image_label.setPixmap(QPixmap.fromImage(q_image).scaled(
                self.image_label.size(), Qt.AspectRatioMode.KeepAspectRatio))

    def start_camera(self):
        """Start camera capture"""
        if not self.camera_running:
            # Thử kết nối lại với camera hiện tại
            self.try_connect_camera(self.current_camera_index)

    def stop_camera(self):
        """Stop camera capture"""
        if self.camera_running:
            # Stop timer
            self.timer.stop()
            self.camera_running = False
            
            # Giải phóng camera nếu có
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap:
                self.ball_detector.cap.release()
            
            # Update button states
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)
            
            # Hiển thị ảnh thông báo camera đã dừng
            self.show_error_image("CAMERA DISCONNECTED")
            
            self.update_status("Camera stopped")

    def update_resolution_display(self):
        """Update the resolution display labels"""
        if hasattr(self, 'resolution_label') and hasattr(self, 'fps_label') and hasattr(self, 'frame_count_label'):
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap and self.ball_detector.cap.isOpened():
                width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
                
                self.resolution_label.setText(f"Resolution: {int(width)}x{int(height)}")
                self.fps_label.setText(f"FPS: {int(fps)}")
            else:
                # Hiển thị giá trị mặc định nếu không có camera
                self.resolution_label.setText(f"Resolution: {int(self.current_width)}x{int(self.current_height)}")
                self.fps_label.setText(f"FPS: {int(self.current_fps)}")
                # Không reset total_frames khi không có camera

    def update_frame(self):
        try:
            # Update frame counters
            self.frame_count += 1
            self.total_frames += 1
            self.frame_skip_counter += 1
            
            # Skip frames để tăng FPS
            if self.frame_skip_counter % self.frame_skip_rate != 0:
                return
            
            # Calculate FPS
            current_time = datetime.now()
            time_diff = (current_time - self.last_fps_update).total_seconds()
            if time_diff >= self.fps_update_interval:
                self.fps = int(self.frame_count / time_diff)
                self.frame_count = 0  # Reset chỉ frame_count để tính FPS
                self.last_fps_update = current_time
                
                # Update FPS display
                self.fps_label.setText(f"FPS: {self.fps}")
            
            # Update total frame count display
            self.frame_count_label.setText(f"Frames: {self.total_frames}")
            
            if not self.camera_running or not self.ball_detector or not hasattr(self.ball_detector, 'cap') or not self.ball_detector.cap or not self.ball_detector.cap.isOpened():
                # Dừng timer nếu camera không còn hoạt động
                if self.timer.isActive():
                    self.timer.stop()
                self.camera_running = False
                
                # Hiển thị thông báo không có camera
                self.show_error_image("COULD NOT CONNECT TO CAMERA")
                
                # Cập nhật trạng thái
                self.update_status("Camera not active", True)
                
                # Cập nhật trạng thái nút
                self.start_camera_button.setEnabled(True)
                self.stop_camera_button.setEnabled(False)
                return
                
            ret, frame = self.ball_detector.cap.read()
            if not ret:
                # Try to reopen camera
                try:
                    self.ball_detector.cap.release()
                    
                    # Thử kết nối lại với camera hiện tại
                    camera_idx = self.current_camera_index
                    self.ball_detector.cap = cv2.VideoCapture(camera_idx)
                    
                    if not self.ball_detector.cap.isOpened():
                        self.update_status("Could not reopen camera", True)
                        self.camera_running = False
                        # Cập nhật trạng thái nút
                        self.start_camera_button.setEnabled(True)
                        self.stop_camera_button.setEnabled(False)
                        # Hiển thị thông báo
                        self.show_error_image(f"COULD NOT REOPEN CAMERA {camera_idx}")
                        return
                        
                except Exception as e:
                    self.update_status(f"Error reopening: {str(e)}", True)
                    self.camera_running = False
                    # Cập nhật trạng thái nút
                    self.start_camera_button.setEnabled(True)
                    self.stop_camera_button.setEnabled(False)
                    # Hiển thị thông báo
                    self.show_error_image("ERROR REOPENING CAMERA")
                    return
                return
                
            frame = cv2.flip(frame, 1)
            
            # Process frame
            cnts, gray = self.ball_detector._find_contours(frame)
            centers, contour_data = self.ball_detector._process_contours(cnts)
            stable_positions = self.ball_detector._update_centroids(centers)
            display_frame, predicted_coordinates = self.ball_detector._draw_on_frame(frame, contour_data, 
                                                            stable_positions)
            
            # Update ball position values if Kalman predicted coordinates are available
            if predicted_coordinates is not None:
                predicted_x, predicted_y = predicted_coordinates
                cm_x, cm_y = self.ball_detector.transformer.image_to_cm_coordinates(predicted_x, predicted_y)
                if cm_x is not None and cm_y is not None:
                    self.current_ball_pos = (cm_x, cm_y)
                    
                    # Update position display on UI
                    if hasattr(self, 'x_value_edit') and hasattr(self, 'y_value_edit'):
                        # Format text
                        x_text = f"{cm_x:.2f} cm"
                        y_text = f"{cm_y:.2f} cm"
                        
                        # Update labels (reduce UI update frequency for performance)
                        self.x_value_edit.setText(x_text)
                        self.y_value_edit.setText(y_text)
                        
                        # Gửi tọa độ qua UART nếu đã đủ thời gian và đã kết nối
                        current_time = time.time()
                        # Gửi ngay lập tức khi có dữ liệu mới để giảm độ trễ
                        if self.serial_comm.is_connected:  # Kiểm tra kết nối từ serial_comm chính của MainWindow
                            # Gửi ngay khi có dữ liệu mới hoặc đã đủ interval
                            should_send = (current_time - self.last_send_time >= self.send_interval)
                            if should_send and self.serial_comm.send_ball_coordinates(cm_x, cm_y):
                                self.last_send_time = current_time
                        
                        # Remove force UI update to improve performance
                        # QApplication.processEvents()
                    
                    if self.values_window is not None:
                        self.values_window.update_data(cm_x, cm_y)
                    
                    # Update status display if available
                    if hasattr(self, 'status_text'):
                        self.status_text.setText("Kalman Tracking")
                        self.status_text.setStyleSheet("color: #27ae60; background-color: #e8f8e8; padding: 8px; border-radius: 3px; font-weight: bold;")
            else:
                # Clear position display when no Kalman predicted coordinates available
                if hasattr(self, 'x_value_edit') and hasattr(self, 'y_value_edit'):
                    self.x_value_edit.setText("--.- cm")
                    self.y_value_edit.setText("--.- cm")
                
                # Update status - no Kalman prediction
                if hasattr(self, 'status_text'):
                    self.status_text.setText("No Kalman Data")
                    self.status_text.setStyleSheet("color: #c0392b; background-color: #fadbd8; padding: 8px; border-radius: 3px; font-weight: bold;")
            
            # Display the image - modified to maintain aspect ratio
            if hasattr(self, 'image_label'):
                height, width, channel = display_frame.shape
                bytes_per_line = 3 * width
                q_image = QImage(display_frame.data, width, height, bytes_per_line, 
                               QImage.Format.Format_RGB888).rgbSwapped()
                
                # Scale image to fit label while maintaining aspect ratio (optimized for speed)
                scaled_pixmap = QPixmap.fromImage(q_image).scaled(
                    self.image_label.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.FastTransformation  # Use fast transformation for better FPS
                )
                
                self.image_label.setPixmap(scaled_pixmap)
        except Exception as e:
            # Update status with error
            if hasattr(self, 'status_text'):
                self.status_text.setText(f"Error: {str(e)}")
                self.status_text.setStyleSheet("color: #c0392b; background-color: #fadbd8; padding: 5px; border-radius: 3px;")

    def toggle_serial_connection(self):
        """Handle serial connection/disconnection"""
        if not self.serial_comm.is_connected:
            # Try to connect
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if self.serial_comm.connect(port, baudrate=baud):
                self.connect_button.setText('Disconnect')
                self.connect_button.setStyleSheet("""
                    QPushButton {
                        background-color: #c0392b;
                        color: white;
                        border-radius: 5px;
                        padding: 8px;
                        font-weight: bold;
                    }
                    QPushButton:hover {
                        background-color: #e74c3c;
                    }
                """)
                self.port_combo.setEnabled(False)
                self.baud_combo.setEnabled(False)
                self.update_status("Serial connected successfully")
            else:
                self.update_status("Failed to connect to serial port", True)
        else:
            # Disconnect
            self.serial_comm.disconnect()
            self.connect_button.setText('Connect')
            self.connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    border-radius: 5px;
                    padding: 8px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #2ecc71;
                }
            """)
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.update_status("Serial disconnected")

    def update_serial_status(self, connected, message):
        """Update serial connection status display"""
        self.update_status(f"Serial: {message}", not connected)

    def closeEvent(self, event):
        # Disconnect serial port before closing
        if self.serial_comm:
            self.serial_comm.disconnect()
        if self.ball_detector and hasattr(self.ball_detector, 'release'):
            self.ball_detector.release()
        if self.values_window:
            self.values_window.close()
        event.accept()





# --- CONTROLLER (application) -----------------------------------------------
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()