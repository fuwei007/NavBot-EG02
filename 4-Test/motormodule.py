'''
Ben Katz
Motor Module Python API
Assumes the serial device is a nucleo running the firmware at:
Corresponding STM32F446 Firmware here:
https://os.mbed.com/users/benkatz/code/CanMaster/
'''
import serial
from struct import *
import time

class MotorModuleController():
	def __init__(self, port):
		try:
			self.ser = serial.Serial(port, timeout = .05)
			self.ser.baudrate = 2000000
			self.rx_data = [0, 0, 0, 0, 0, 0]
			self.rx_values = [[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]
			self.tx_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
			self.off_set = [0, 0, 0, 0,]
			print('connected to motor module controller')
		except:
			print('failed to connect to motor module controller')
			pass
	def send_data(self):
		pass
	def mysend_command(self,id, p_des, v_des, kp, kd, i_ff):
		"""
		self, id, p_des, v_des, kp, kd, i_ff
		Generate 20 bytes of robot control data. 
		Parameters:
		id_value: Byte 6, ID (0-255)
		position: Bytes 11-12, Position parameter (0-65535)
		v_des: The first 12-bit parameter in bytes 13-18, Speed (0-4095)
		kp: The second 12-bit parameter in bytes 13-18, Proportional coefficient (0-4095)
		kd: The third 12-bit parameter in bytes 13-18, Differential coefficient (0-4095)
		i_ff: The fourth 12-bit parameter in bytes 13-18, Feedforward control (0-4095)
		A 20-byte hexadecimal string
		"""
		# Verify the parameter range.
		if not (0 <= id <= 255):
			raise ValueError("The ID must be within the range of 0 to 255.")
		if not (0 <= p_des <= 65535):
			raise ValueError("The value of p_des must be within the range of 0 to 65535.")
		if not all(0 <= x <= 4095 for x in [v_des, kp, kd, i_ff]):
			raise ValueError("The values of v_des, kp, kd and i_ff must be within the range of 0 to 4095.")
		
		# The first 5 bytes of fixed data
		fixed_header = bytes.fromhex('AA 55 01 01 01')
		
		# Byte 6: ID
		id_byte = bytes([id])
		
		# The 7th to 10th bytes of fixed data
		fixed_middle = bytes.fromhex('00 00 00 08')
		
		# Bytes 11-12: position (with the high byte first)
		position_bytes = p_des.to_bytes(2, 'big')
		
		# Bytes 13-18: 4 12-bit parameters (each parameter occupies 1.5 bytes)
		# Pack the 4 12-bit parameters into 6 bytes
		param_data = 0
		param_data = (param_data << 12) | (v_des & 0xFFF)
		param_data = (param_data << 12) | (kp & 0xFFF)
		param_data = (param_data << 12) | (kd & 0xFFF)
		param_data = (param_data << 12) | (i_ff & 0xFFF)
		
		# Converted to 6-byte format (high byte first)
		param_bytes = param_data.to_bytes(6, 'big')
		
		# Fixed Data at Byte 19
		fixed_end = bytes.fromhex('00')
		
		# The first 19 bytes before combination
		all_data = fixed_header + id_byte + fixed_middle + position_bytes + param_bytes + fixed_end
		
		# Calculate the checksum: Sum of the first 19 bytes + 1, and then take the lower 8 bits.
		checksum = (sum(all_data) + 1) & 0xFF
		checksum_byte = bytes([checksum])
		
		# Combine the complete data
		final_data = all_data + checksum_byte
		
		self.ser.reset_input_buffer()
		self.ser.write(final_data)
		# time.sleep(.02)
		b_rx = self.ser.read(20)
		hex_str = b_rx.hex().upper()
		
		self.rx_data = b_rx[10:16]
		r_id = b_rx[10]
		self.rx_values[r_id][0] = b_rx[10];					# ID
		self.rx_values[r_id][1] = (int.from_bytes(b_rx[11:13], byteorder='big')) &0xffff		# Position
		self.rx_values[r_id][2] = int.from_bytes(b_rx[13:15], byteorder='big') >> 4		# Velocity
		self.rx_values[r_id][3] = int.from_bytes(b_rx[14:16], byteorder='big') & 0xfff	# Current
		# print(hex_str)

	def enable_motor(self, id):
		"""
		Puts motor with CAN ID "id" into torque-control mode.  2nd red LED will turn on
		"""
		# b = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
		# b = b + bytes(bytearray([id]))

		# The first 5 bytes of fixed data
		fixed_header = bytes.fromhex('AA 55 01 01 01')
		
		
		id_byte = bytes([id])
		
		
		fixed_middle = bytes.fromhex('00 00 00 08 FF FF FF FF FF FF FF FC 00')
		

		all_data = fixed_header + id_byte + fixed_middle
		

		checksum = (sum(all_data) + 1) & 0xFF
		checksum_byte = bytes([checksum])
	
		
		b = all_data + checksum_byte


		self.ser.reset_input_buffer()
		self.ser.write(b)


		b_rx = self.ser.read(20)
		self.rx_data = b_rx[10:16]
		b_p = b_rx[11:13]
		# hex_str = b_p.hex().upper()
		# print(hex_str)
		r_id = b_rx[10]
		self.rx_values[r_id][0] = r_id				# ID
		self.rx_values[r_id][1] = (int.from_bytes(b_rx[11:13], byteorder='big')) &0xffff		# Position
		self.rx_values[r_id][2] = int.from_bytes(b_rx[13:15], byteorder='big') >> 4		# Velocity
		self.rx_values[r_id][3] = int.from_bytes(b_rx[14:16], byteorder='big') & 0xfff	# Current

		#self.ser.flushInput()
	def disable_motor(self, id):
		"""
		Removes motor with CAN ID "id" from torque-control mode.  2nd red LED will turn off
		"""
		
		fixed_header = bytes.fromhex('AA 55 01 01 01')
		
		
		id_byte = bytes([id])
		
		
		fixed_middle = bytes.fromhex('00 00 00 08 FF FF FF FF FF FF FF FD 00')
		
		
		all_data = fixed_header + id_byte + fixed_middle
		
		
		checksum = (sum(all_data) + 1) & 0xFF
		checksum_byte = bytes([checksum])
		
		
		b = all_data + checksum_byte


		self.ser.reset_input_buffer()
		self.ser.write(b)
		time.sleep(0.1)

		b_rx = self.ser.read(20)
		self.rx_data = b_rx[10:16]
		# hex_str = self.rx_data.hex().upper()
		# print(self.rx_values[id],hex_str)
		r_id = b_rx[10]
		self.rx_values[r_id][0] = b_rx[10];					# ID
		self.rx_values[r_id][1] = (int.from_bytes(b_rx[11:13], byteorder='big')) &0xffff		# Position
		self.rx_values[r_id][2] = int.from_bytes(b_rx[13:15], byteorder='big') >> 4		# Velocity
		self.rx_values[r_id][3] = int.from_bytes(b_rx[14:16], byteorder='big') & 0xfff	# Current
	def zero_motor(self, id):
		
		fixed_header = bytes.fromhex('AA 55 01 01 01')
		
		
		id_byte = bytes([id])
		
		
		fixed_middle = bytes.fromhex('00 00 00 08 FF FF FF FF FF FF FF FE 00')
		
		
		all_data = fixed_header + id_byte + fixed_middle
		
		
		checksum = (sum(all_data) + 1) & 0xFF
		checksum_byte = bytes([checksum])
		
		
		b = all_data + checksum_byte


		self.ser.reset_input_buffer()
		self.ser.write(b)
		time.sleep(0.1)


