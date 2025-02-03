import json
import serial
import time
import threading
from enum import Enum, unique

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Author: Karl.Kwon
# Email: mrthinks@gmail.com

mm_2_step = 10.5

@unique
class ConveyorState(Enum):
	INIT = 0
	READY = 1
	RUN = 2
	DISCONNECT = 3

class Worker(threading.Thread):
	def __init__(self, pub):
		super().__init__()
		self.ser = None
		self.is_run = threading.Event()
		self.state = ConveyorState.DISCONNECT
		self.state_past = self.state
		self.publisher_ = pub

		self.publish_status('DISCONNECT')

	def end(self):
		self.is_run.set()

	def reconnect(self):
		try:
			self.ser = serial.Serial('/dev/ttyACM1', baudrate=115200, timeout=1)
			print("Reconnected to serial port.")

			self.state = ConveyorState.INIT
			self.publish_status('INIT')
		except serial.SerialException as e:
			print(f"Error reconnecting: {e}")
			if self.state != ConveyorState.DISCONNECT:
				self.publish_status('DISCONNECT')
			self.state = ConveyorState.DISCONNECT
			self.ser = None
		finally:
			pass

	def write(self, bytes_string):
		if self.ser and self.ser.is_open:
			try:
				self.ser.write(bytes_string)
			except serial.SerialException as e:
				print(f"Error reconnecting: {e}")
				self.state = ConveyorState.DISCONNECT
				self.ser = None
		else:
			self.reconnect()


	def publish_status(self, str_):
		msg = String()
		msg.data = str_
		self.publisher_.publish(msg)
		print(self.state)

	def run(self):
		while not self.is_run.is_set():
			if self.ser and self.ser.is_open:
				try:
					data = self.ser.read()

					if data == b's' or data == b'.':
						self.state = ConveyorState.READY

						if self.state != self.state_past:
							self.publish_status('READY')

						self.state_past = self.state
					elif data == b'_':
						self.state = ConveyorState.RUN

						if self.state != self.state_past:
							self.publish_status('RUN')

						self.state_past = self.state
				except serial.SerialException as e:
					print(f"Error reconnecting: {e}")
					self.state = ConveyorState.DISCONNECT
					self.ser = None
					self.publish_status('DISCONNECT')
			else:
				self.reconnect()
				time.sleep(0.5)


class ConveyorNode(Node):
	def __init__(self):
		super().__init__('image_publisher')

		self.subscription_ = self.create_subscription(String, 'conveyor/control', self.listener_callback_serial, 10)
		self.publisher_ = self.create_publisher(String, 'conveyor/status', 10)

		self.t1 = Worker(self.publisher_)
		self.t1.start()

	def listener_callback_serial(self, msg):
		print('listener_callback_serial')
		if self.t1.state != ConveyorState.INIT:
			data2 = json.loads(msg.data)
			print(data2)

			if data2['control'] == 'go':
				# input_string = '%d ' % data2['distance.mm']
				# bytes_string = input_string.encode('utf-8')
				steps = int(int(data2['distance.mm']) * mm_2_step)
				# self.ser.write(bytes_string)
				self.t1.write((str(steps) + '.').encode('utf-8'))
			elif data2['control'] == 'stop':
				print('stop')
				# self.t1.write(('0.').encode('utf-8'))
				self.t1.write(b'1.')

	def release(self):
		self.t1.end()
		self.t1.join()


# '{"control":"go", "distance.mm": 200}'
# ros2 topic echo /conveyor/status

def main(args=None):
	rclpy.init(args=args)

	conveyor_subscriber = ConveyorNode()
	rclpy.spin(conveyor_subscriber)

	conveyor_subscriber.release()
	conveyor_subscriber.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':

	main()
