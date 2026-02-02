import serial
import struct
import time
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal

class SerialCommunication(QObject):
    # Signals for GUI updates
    connection_status = pyqtSignal(bool, str)  # Connected/Disconnected, Message
    data_received = pyqtSignal(str)  # Data received from STM32
    
    # Protocol constants - Giữ nguyên giao thức cũ theo yêu cầu
    START_BIT = 0x40    # '@' character (64 decimal)
    STOP_BIT = 0x58     # 'X' character (88 decimal)
    
    # Coordinate constants
    COORDINATE_SCALE = 10  # Multiply coordinates by 10 to keep 1 decimal place
    MAX_COORDINATE = 9.5   # Limit coordinates to ±9.5cm
    
    # Angle constants
    ANGLE_SCALE = 100      # Multiply angles by 100 to keep 2 decimal places
    MAX_ANGLE = 30.0       # Limit angles to ±30 degrees
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.port_name = None
        
        # Default serial parameters - Optimized for low latency
        self.default_settings = {
            'baudrate': 115200,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_NONE,
            'stopbits': serial.STOPBITS_ONE,
            'timeout': 0.001,      # Reduced from 0.1 to 1ms for faster response
            'write_timeout': 0.005  # Reduced from 0.1 to 5ms for faster writes
        }
        
        # Buffer for incoming data
        self.receive_buffer = bytearray()
    
    def calculate_checksum(self, data):
        """
        Calculate CheckSum by summing decimal values
        from start bit to stop bit
        Args:
            data: bytearray containing data from start_bit to stop_bit
        Returns:
            int: checksum value
        """
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFFFF  # 16-bit checksum
    
    def pack_data_frame(self, parameter_data):
        """
        Pack data in format: <Start_bit><Parameter><Stop_bit><CheckSum>
        Args:
            parameter_data: bytes or bytearray containing parameter data
        Returns:
            bytearray: Packed frame
        """
        if isinstance(parameter_data, str):
            parameter_data = parameter_data.encode('ascii')
        elif not isinstance(parameter_data, (bytes, bytearray)):
            parameter_data = bytes(parameter_data)
        
        # Create frame in format
        frame = bytearray()
        frame.append(self.START_BIT)        # Start bit '@'
        frame.extend(parameter_data)        # Parameter data
        frame.append(self.STOP_BIT)         # Stop bit 'X'
        
        # Calculate checksum from start bit to stop bit
        checksum_data = frame[:]  # Copy entire frame from start to stop
        checksum = self.calculate_checksum(checksum_data)
        
        # Add checksum (2 bytes, little-endian)
        frame.extend(struct.pack('<H', checksum))
        
        return frame
    
    def send_packed_frame(self, parameter_data):
        """
        Send packed data in frame format
        Args:
            parameter_data: Parameter data to send
        Returns:
            bool: True if sent successfully
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            packed_frame = self.pack_data_frame(parameter_data)
            self.serial_port.write(packed_frame)
            # Remove flush() for better performance - OS will handle buffering
            # self.serial_port.flush()
            return True
        except Exception as e:
            self.connection_status.emit(False, f"Send error: {str(e)}")
            return False
    
    def send_ball_coordinates(self, x, y):
        """
        Send x, y coordinates of the ball in frame format
        Use int8 (-128 to 127) to optimize packet size
        Args:
            x: x coordinate of the ball (float, cm, -9.5 to 9.5)
            y: y coordinate of the ball (float, cm, -9.5 to 9.5)
        Returns:
            bool: True if sent successfully
        """
        # Check coordinate limits
        if abs(x) > self.MAX_COORDINATE or abs(y) > self.MAX_COORDINATE:
            return False
        
        # Convert coordinates to integer (multiply by 10)
        x_int = int(round(x * self.COORDINATE_SCALE))
        y_int = int(round(y * self.COORDINATE_SCALE))
        
        # Pack coordinates into 2 bytes (int8 + int8) - NO DATA TYPE
        coord_data = struct.pack('<bb', x_int, y_int)
        
        # Send frame
        return self.send_packed_frame(coord_data)
    
    def send_string_frame(self, message):
        """
        Send text string in frame format
        Args:
            message: Text string to send
        Returns:
            bool: True if sent successfully
        """
        return self.send_packed_frame(message)
    
    def unpack_data_frame(self, frame):
        """
        Unpack frame and check integrity
        Args:
            frame: bytearray containing received frame
        Returns:
            tuple: (success: bool, parameter_data: bytes, error_msg: str)
        """
        try:
            if len(frame) < 4:  # Minimum: START + STOP + CHECKSUM(2 bytes)
                return False, None, "Frame too short"
            
            # Check start bit
            if frame[0] != self.START_BIT:
                return False, None, f"Invalid start bit. Expected: {self.START_BIT}, Got: {frame[0]}"
            
            # Find stop bit
            stop_index = -1
            for i in range(1, len(frame) - 2):  # -2 to leave room for checksum
                if frame[i] == self.STOP_BIT:
                    stop_index = i
                    break
            
            if stop_index == -1:
                return False, None, "Stop bit not found"
            
            # Check frame length (must have enough for checksum)
            if len(frame) < stop_index + 3:  # stop_index + 1 + 2 bytes checksum
                return False, None, "Frame too short for checksum"
            
            # Extract parameter data
            parameter_data = frame[1:stop_index]
            
            # Extract checksum (2 bytes after stop bit)
            received_checksum = struct.unpack('<H', frame[stop_index+1:stop_index+3])[0]
            
            # Calculate checksum for verification (from start bit to stop bit)
            checksum_data = frame[:stop_index+1]
            calculated_checksum = self.calculate_checksum(checksum_data)
            
            # Check checksum
            if received_checksum != calculated_checksum:
                return False, None, f"Checksum mismatch. Expected: {calculated_checksum}, Got: {received_checksum}"
            
            return True, bytes(parameter_data), "Success"
            
        except Exception as e:
            return False, None, f"Unpack error: {str(e)}"
    
    def read_and_process_frames(self):
        """
        Read and process frames received from serial port
        Returns:
            list: List of valid parameter data decoded
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return []
        
        valid_frames = []
        
        try:
            # Read available data
            if self.serial_port.in_waiting > 0:
                new_data = self.serial_port.read(self.serial_port.in_waiting)
                self.receive_buffer.extend(new_data)
            
            # Find and process complete frames
            while len(self.receive_buffer) >= 4:  # Minimum frame size
                # Find start bit
                start_index = -1
                for i in range(len(self.receive_buffer)):
                    if self.receive_buffer[i] == self.START_BIT:
                        start_index = i
                        break
                
                if start_index == -1:
                    # No start bit found, clear buffer
                    self.receive_buffer.clear()
                    break
                
                # Remove data before start bit
                if start_index > 0:
                    self.receive_buffer = self.receive_buffer[start_index:]
                
                # Find stop bit
                stop_index = -1
                for i in range(1, len(self.receive_buffer) - 2):  # -2 to leave room for checksum
                    if self.receive_buffer[i] == self.STOP_BIT:
                        stop_index = i
                        break
                
                if stop_index == -1:
                    # Stop bit not found yet, need more data
                    break
                
                # Check if enough data for checksum
                frame_end = stop_index + 3  # stop_index + 1 + 2 bytes checksum
                if len(self.receive_buffer) < frame_end:
                    break
                
                # Extract complete frame
                frame = self.receive_buffer[:frame_end]
                self.receive_buffer = self.receive_buffer[frame_end:]
                
                # Decode frame
                success, parameter_data, error_msg = self.unpack_data_frame(frame)
                if success:
                    # Decode based on data type
                    decoded_data = self.decode_parameter_data(parameter_data)
                    if decoded_data:
                        valid_frames.append(decoded_data)
                        self.data_received.emit(decoded_data)
                    else:
                        pass
                else:
                    pass
                    
        except Exception as e:
            pass
        
        return valid_frames

    def decode_parameter_data(self, parameter_data):
        """
        Decode parameter data based on frame length only (no data type identifiers)
        Args:
            parameter_data: bytes containing the parameter data
        Returns:
            str: Formatted string for display, or None if unknown format
        """
        if len(parameter_data) < 1:
            return None
        
        try:
            # Detect frame type based on length
            if len(parameter_data) == 2:
                # Ball coordinates (x + y) - 2 bytes
                x_int, y_int = struct.unpack('<bb', parameter_data)
                x = x_int / self.COORDINATE_SCALE
                y = y_int / self.COORDINATE_SCALE
                return f"Ball Position: X={x:.1f}cm, Y={y:.1f}cm"
                
            elif len(parameter_data) == 4:
                # Angles only (theta + phi) - 4 bytes
                theta_int, phi_int = struct.unpack('<hh', parameter_data)
                theta = theta_int / self.ANGLE_SCALE
                phi = phi_int / self.ANGLE_SCALE
                return f"Angles: θ={theta:.2f}°, φ={phi:.2f}°"
                
            elif len(parameter_data) == 10:
                # Angles and PWM for 3 servos (theta + phi + pwm1 + pwm2 + pwm3) - 10 bytes
                theta_int, phi_int, pwm_servo1, pwm_servo2, pwm_servo3 = struct.unpack('<hhHHH', parameter_data)
                theta = theta_int / self.ANGLE_SCALE
                phi = phi_int / self.ANGLE_SCALE
                return f"Angles: θ={theta:.2f}°, φ={phi:.2f}° | PWM: S1={pwm_servo1}, S2={pwm_servo2}, S3={pwm_servo3}"
            
            # If we reach here, unknown frame length
            return f"Unknown frame length: {len(parameter_data)} bytes | Data: {parameter_data.hex()}"
            
        except Exception as e:
            return f"Decode error: {str(e)} | Raw: {parameter_data.hex()}"

    def get_available_ports(self):
        """Get list of available COM ports from Device Manager"""
        ports = []
        try:
            # Scan all available COM ports in the system
            available_ports = serial.tools.list_ports.comports()
            
            # Filter and add detailed info for each port
            for port in available_ports:
                # Add COM port name (e.g.: COM6)
                ports.append(port.device)
                print(f"Found port: {port.device}")
                # Print more details for debugging
                if port.description:
                    print(f"  Description: {port.description}")
                if port.manufacturer:
                    print(f"  Manufacturer: {port.manufacturer}")
                if port.hwid:
                    print(f"  Hardware ID: {port.hwid}")
            
            # If no ports found, return empty list
            if not ports:
                print("No COM ports found in system")
                
        except Exception as e:
            print(f"Error scanning COM ports: {str(e)}")
            
        return sorted(ports)  # Sort ports alphabetically
    
    def connect(self, port_name, baudrate=115200, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE):
        """
        Connect to specified COM port with given parameters
        Returns: True if successful, False otherwise
        """
        try:
            if self.is_connected:
                self.disconnect()
                
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=self.default_settings['timeout'],
                write_timeout=self.default_settings['write_timeout']
            )
            
            if self.serial_port.is_open:
                self.is_connected = True
                self.port_name = port_name
                self.connection_status.emit(True, f"Connected to {port_name}")
                return True
            else:
                self.connection_status.emit(False, f"Failed to open {port_name}")
                return False
                
        except Exception as e:
            self.connection_status.emit(False, f"Connection error: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from current COM port"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                pass
            finally:
                self.is_connected = False
                self.port_name = None
                self.receive_buffer.clear()
                self.connection_status.emit(False, "Disconnected")
    
    def read_data(self):
        """
        Read raw data from serial port for Hercules terminal
        Returns: decoded string if data available, empty string otherwise
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return ""
        
        try:
            if self.serial_port.in_waiting > 0:
                raw_data = self.serial_port.read(self.serial_port.in_waiting)
                # Try to decode as UTF-8, fallback to latin-1 if fails
                try:
                    return raw_data.decode('utf-8')
                except UnicodeDecodeError:
                    return raw_data.decode('latin-1', errors='replace')
            return ""
        except Exception as e:
            return ""
    
    def __del__(self):
        """Destructor to ensure port is closed"""
        self.disconnect() 