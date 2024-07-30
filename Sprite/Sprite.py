import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3

intialize_flip = False
flip_frame = 0

# https://python-statemachine.readthedocs.io/en/latest/readme.html

class RLSprite(BaseAgent):

	def initialize_agent(self):
		# This runs once before the bot starts up
		self.controller_state = SimpleControllerState()
	
	def boost(self, target_x, target_y, boost_reserve):
		
		global intialize_flip
		
		x_distance_to_target = (self.bot_pos.x - target_x)
		y_distance_to_target = (self.bot_pos.y - target_y)
		planer_distance_to_target = math.hypot(int(x_distance_to_target), int(y_distance_to_target))
		
		if (target_x == 0 and target_y < 100 and target_y > 90) and planer_distance_to_target > 100:
			self.controller_state.boost = True
		elif planer_distance_to_target > 1000 and intialize_flip == False and self.bot_boost > boost_reserve:
			self.controller_state.boost = True
		else:
			self.controller_state.boost = False
	
	def check_flip(self, target_x, target_y, target_z):
	
		global intialize_flip
		global flip_frame
		global car_facing_ball
	
		x_distance_to_target = (self.bot_pos.x - target_x)
		y_distance_to_target = (self.bot_pos.y - target_y)
		distance_to_target = math.hypot(int(x_distance_to_target), int(y_distance_to_target))
	
		if distance_to_target < 1000 and ((target_z - 73) - self.bot_pos.z) < 75:
			intialize_flip = True
	
		if intialize_flip == True:
			if flip_frame == 0:
				self.controller_state.jump = True
				flip_frame += 1
			elif flip_frame == 10:
				self.controller_state.jump = False
				flip_frame += 1
			elif flip_frame == 15:
				self.controller_state.jump = True
				flip_frame += 1
			elif flip_frame == 25:
				self.controller_state.jump = False
				flip_frame += 1
			elif flip_frame == 80:
				flip_frame = 0
				intialize_flip = False
			else:
				flip_frame += 1
			
			if flip_frame > 15:
				self.controller_state.pitch = -1
			else:
				self.controller_state.pitch = 0
	
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
		global car_facing_ball
		
		ball_location = Vec3(packet.game_ball.physics.location)

		my_car = packet.game_cars[self.index]
		car_location = Vec3(my_car.physics.location)
		car_to_ball = ball_location - car_location
		ball_pos = packet.game_ball.physics.location
		self.bot_wheels = packet.game_cars[self.index].has_wheel_contact
		self.bot_boost = packet.game_cars[self.index].boost
		self.bot_roll = packet.game_cars[self.index].physics.rotation.roll

		# Find the direction of our car using the Orientation class
		car_orientation = Orientation(my_car.physics.rotation)
		car_direction = car_orientation.forward
		self.bot_pos = packet.game_cars[self.index].physics.location
		
		steer_correction_radians = find_correction(car_direction, car_to_ball)

		if steer_correction_radians > 0:
			# Positive radians in the unit circle is a turn to the left.
			turn = -1.0  # Negative value for a turn to the left.
			action_display = "turn left"
		else:
			turn = 1.0
			action_display = "turn right"

		self.controller_state.throttle = 1.0
		self.controller_state.steer = turn
		self.check_flip(ball_pos.x, ball_pos.y, ball_pos.z)
		self.boost(ball_pos.x, ball_pos.y, 0)
		#draw_debug(self.renderer, my_car, packet.game_ball, action_display)

		return self.controller_state


def find_correction(current: Vec3, ideal: Vec3) -> float:
	# Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

	# The in-game axes are left handed, so use -x
	current_in_radians = math.atan2(current.y, -current.x)
	ideal_in_radians = math.atan2(ideal.y, -ideal.x)

	diff = ideal_in_radians - current_in_radians

	# Make sure that diff is between -pi and +pi.
	if abs(diff) > math.pi:
		if diff < 0:
			diff += 2 * math.pi
		else:
			diff -= 2 * math.pi

	return diff


def draw_debug(renderer, car, ball, action_display):
	renderer.begin_rendering()
	# draw a line from the car to the ball
	renderer.draw_line_3d(car.physics.location, ball.physics.location, renderer.white())
	# print the action that the bot is taking
	renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
	renderer.end_rendering()