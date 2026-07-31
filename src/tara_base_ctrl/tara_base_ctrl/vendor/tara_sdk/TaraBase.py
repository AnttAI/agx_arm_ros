import minimalmodbus
import logging
import struct

class TaraBase:
    def __init__(self, port='/dev/ttyUSB0', slave_id=1, baudrate=115200, debug=False):
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.debug = debug
        self.instrument = None
        self.connected = False
        
        # Configure logging for debug mode
        if self.debug:
            logging.basicConfig(level=logging.DEBUG)
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.INFO)

    def connect(self):
        if not self.connected:
            try:
                self.instrument = minimalmodbus.Instrument(self.port, self.slave_id)
                self.instrument.serial.baudrate = self.baudrate
                self.instrument.serial.bytesize = 8
                self.instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
                self.instrument.serial.stopbits = 1
                self.instrument.serial.timeout = 1
                self.instrument.mode = minimalmodbus.MODE_RTU
                
                # Enable debug mode if requested
                if self.debug:
                    self.instrument.debug = True
                    self.logger.debug(f"MODBUS debug enabled for {self.port}")
                
                # Test connection by reading a register
                if self.debug:
                    self.logger.debug("Testing connection by reading control register 0x200E")
                self.instrument.read_register(0x200E)
                
                self.connected = True
                print(f"TaraBase connected to {self.port} (slave {self.slave_id})")
                
                # Initialize velocity control mode
                self._initialize_velocity_control()
                
            except Exception as e:
                print(f"Connection failed: {e}")
                self.connected = False
        else:
            print("Already connected.")

    def _initialize_velocity_control(self):
        if self.debug:
            self.logger.debug("Initializing velocity control mode")
        
        # Set profile velocity mode (3)
        self.instrument.write_register(0x200D, 3, functioncode=6)
        
        # Set default acceleration/deceleration times (500ms)
        self.instrument.write_register(0x2080, 500, functioncode=6)  # Left acceleration
        self.instrument.write_register(0x2081, 500, functioncode=6)  # Right acceleration  
        self.instrument.write_register(0x2082, 500, functioncode=6)  # Left deceleration
        self.instrument.write_register(0x2083, 500, functioncode=6)  # Right deceleration
        
        if self.debug:
            self.logger.debug("Velocity control mode initialized")

    def set_velocity(self, left_rpm, right_rpm):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        # Validate input range (-3000 to 3000 RPM)
        if not (-3000 <= left_rpm <= 3000) or not (-3000 <= right_rpm <= 3000):
            print("Target velocity must be between -3000 and 3000 RPM")
            return False
        
        try:
            if self.debug:
                self.logger.debug(f"Setting velocities: left={left_rpm} RPM, right={right_rpm} RPM")
            
            # Convert signed values to unsigned 16-bit for MODBUS
            left_unsigned = left_rpm if left_rpm >= 0 else left_rpm + 65536
            right_unsigned = right_rpm if right_rpm >= 0 else right_rpm + 65536
            
            if self.debug:
                self.logger.debug(f"Converted to unsigned: left={left_unsigned}, right={right_unsigned}")
            
            # Enable motors
            self.instrument.write_register(0x200E, 0x08, functioncode=6)
            
            # The ZLAC8015D manual calls this "Synchronous velocity control":
            # write left (0x2088) and right (0x2089) in one function-16 frame.
            self.instrument.write_registers(0x2088, [left_unsigned, right_unsigned])

            print(f"Set velocities - Left: {left_rpm} RPM, Right: {right_rpm} RPM")
            return True
            
        except Exception as e:
            print(f"Failed to set velocity: {e}")
            if self.debug:
                self.logger.error(f"set_velocity failed: {e}")
            return False

    def get_actual_velocity(self):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return None
        
        try:
            if self.debug:
                self.logger.debug("Reading actual velocities...")
            
            # Read actual velocities (unit: 0.1r/min)
            left_vel_raw = self.instrument.read_register(0x20AB)
            right_vel_raw = self.instrument.read_register(0x20AC)
            
            # Convert from unsigned to signed 16-bit if negative
            if left_vel_raw >= 0x8000:
                left_vel_raw -= 0x10000
            if right_vel_raw >= 0x8000:
                right_vel_raw -= 0x10000
            
            # Convert from 0.1r/min to r/min
            left_velocity = left_vel_raw * 0.1
            right_velocity = right_vel_raw * 0.1
            
            velocity_data = {
                'left_motor_velocity': left_velocity,
                'right_motor_velocity': right_velocity
            }
            
            if self.debug:
                self.logger.debug(f"Actual velocities - Left: {left_velocity} RPM, Right: {right_velocity} RPM")
            
            return velocity_data
            
        except Exception as e:
            print(f"Failed to read velocities: {e}")
            if self.debug:
                self.logger.error(f"get_actual_velocity failed: {e}")
            return None

    def get_motor_positions(self):
        """Return signed 32-bit left/right encoder positions in controller counts."""
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return None

        try:
            # Read both motors in one MODBUS transaction. Registers 0x20A7..0x20AA
            # are left-high, left-low, right-high, right-low.
            registers = self.instrument.read_registers(0x20A7, 4, functioncode=3)
            left_position = struct.unpack(">i", struct.pack(">HH", *registers[:2]))[0]
            right_position = struct.unpack(">i", struct.pack(">HH", *registers[2:]))[0]
            return {
                "left_motor_position": left_position,
                "right_motor_position": right_position,
            }
        except Exception as e:
            print(f"Failed to read motor positions: {e}")
            if self.debug:
                self.logger.error(f"get_motor_positions failed: {e}")
            return None

    def stop_motors(self):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        try:
            if self.debug:
                self.logger.debug("Stopping motors")
            
            # Send stop command
            self.instrument.write_register(0x200E, 0x07, functioncode=6)
            print("Motors stopped")
            return True
            
        except Exception as e:
            print(f"Failed to stop motors: {e}")
            if self.debug:
                self.logger.error(f"stop_motors failed: {e}")
            return False

    def enable_motors(self):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        try:
            if self.debug:
                self.logger.debug("Enabling motors")
            
            # Send enable command
            self.instrument.write_register(0x200E, 0x08, functioncode=6)
            print("Motors enabled")
            return True
            
        except Exception as e:
            print(f"Failed to enable motors: {e}")
            if self.debug:
                self.logger.error(f"enable_motors failed: {e}")
            return False

    def emergency_stop(self):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        try:
            if self.debug:
                self.logger.debug("Emergency stop activated")
            
            # Send emergency stop command
            self.instrument.write_register(0x200E, 0x05, functioncode=6)
            print("EMERGENCY STOP ACTIVATED")
            return True
            
        except Exception as e:
            print(f"Failed to execute emergency stop: {e}")
            if self.debug:
                self.logger.error(f"emergency_stop failed: {e}")
            return False

    def clear_fault(self):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        try:
            if self.debug:
                self.logger.debug("Clearing faults")
            
            # Send clear fault command
            self.instrument.write_register(0x200E, 0x06, functioncode=6)
            print("Faults cleared")
            return True
            
        except Exception as e:
            print(f"Failed to clear faults: {e}")
            if self.debug:
                self.logger.error(f"clear_fault failed: {e}")
            return False

    def set_acceleration_time(self, left_ms, right_ms):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        # Validate range (0-32767ms)
        if not (0 <= left_ms <= 32767) or not (0 <= right_ms <= 32767):
            print("Acceleration time must be between 0 and 32767 ms")
            return False
        
        try:
            if self.debug:
                self.logger.debug(f"Setting acceleration times: left={left_ms}ms, right={right_ms}ms")
            
            self.instrument.write_register(0x2080, left_ms, functioncode=6)   # Left acceleration
            self.instrument.write_register(0x2081, right_ms, functioncode=6)  # Right acceleration
            
            print(f"Set acceleration times - Left: {left_ms}ms, Right: {right_ms}ms")
            return True
            
        except Exception as e:
            print(f"Failed to set acceleration times: {e}")
            if self.debug:
                self.logger.error(f"set_acceleration_time failed: {e}")
            return False

    def set_deceleration_time(self, left_ms, right_ms):
        if not self.connected or not self.instrument:
            print("Robot not connected.")
            return False
        
        # Validate range (0-32767ms)
        if not (0 <= left_ms <= 32767) or not (0 <= right_ms <= 32767):
            print("Deceleration time must be between 0 and 32767 ms")
            return False
        
        try:
            if self.debug:
                self.logger.debug(f"Setting deceleration times: left={left_ms}ms, right={right_ms}ms")
            
            self.instrument.write_register(0x2082, left_ms, functioncode=6)   # Left deceleration
            self.instrument.write_register(0x2083, right_ms, functioncode=6)  # Right deceleration
            
            print(f"Set deceleration times - Left: {left_ms}ms, Right: {right_ms}ms")
            return True
            
        except Exception as e:
            print(f"Failed to set deceleration times: {e}")
            if self.debug:
                self.logger.error(f"set_deceleration_time failed: {e}")
            return False

    def disconnect(self):
        if self.connected:
            # Send stop command to motors before disconnecting
            if self.instrument and self.instrument.serial.is_open:
                try:
                    if self.debug:
                        self.logger.debug("Sending stop command to motors before disconnect")
                    self.instrument.write_register(0x200E, 0x07, functioncode=6)  # Stop command
                except Exception as e:
                    print(f"Warning: Failed to send stop command during disconnect: {e}")
                    if self.debug:
                        self.logger.error(f"Stop command failed: {e}")
                
                # Close the serial connection
                self.instrument.serial.close()
            self.connected = False
            print("TaraBase disconnected.")
        else:
            print("Robot already disconnected.")
