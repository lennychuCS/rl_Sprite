import math
from enum import Enum

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3

# https://python-statemachine.readthedocs.io/en/latest/readme.html
o_goal_locations = [Vec3(0, 5120, 0), Vec3(0, -5120, 0)]
d_goal_locations = [Vec3(0, -5120, 0), Vec3(0, 5120, 0)]

class State(Enum):
	KICKOFF = 0
	OFFENSE = 1
	DEFENSE = 2
	
class RLSprite(BaseAgent):

	def initialize_agent(self):
		# This runs once before the bot starts up
		self.controller_state = SimpleControllerState()
	
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
		global car_facing_ball
		
		ball_location = Vec3(packet.game_ball.physics.location)

		my_car = packet.game_cars[self.index]
		car_location = Vec3(my_car.physics.location)
		car_to_ball = ball_location - car_location
		car_to_o_goal = o_goal_locations[self.team] - car_location

		distance_to_ball = car_to_ball.flat().length()

		self.bot_wheels = packet.game_cars[self.index].has_wheel_contact
		self.bot_boost = packet.game_cars[self.index].boost
		self.bot_roll = packet.game_cars[self.index].physics.rotation.roll

		# Find the direction of our car using the Orientation class
		car_orientation = Orientation(my_car.physics.rotation)
		car_direction = car_orientation.forward
		self.bot_pos = packet.game_cars[self.index].physics.location

		desired_speed = 0
		current_speed = Vec3(packet.game_cars[self.index].physics.velocity).flat().length()
		ball_speed = Vec3(packet.game_ball.physics.velocity).flat().length()

		# Steer Correction Radians
		ball_scr = find_correction(car_direction, car_to_ball)
		o_goal_scr = find_correction(car_direction, car_to_o_goal)

		if(distance_to_ball > 1000):
			desired_speed = 1500
		elif(distance_to_ball > 500):
			desired_speed = 1200
		elif(distance_to_ball > 200):
			desired_speed = clamp(1000, ball_speed - 200, ball_speed + 800)
		else:
			desired_speed = clamp(8000, ball_speed - 200, ball_speed + 800)

		s_threash = 0.05

		if ball_scr > (0 + s_threash):
			# Positive radians in the unit circle is a turn to the left.
			turn = -min(abs(ball_scr), 1.0)  # Negative value for a turn to the left.
			action_display = "turn left"
		else:
			turn = min(abs(ball_scr), 1.0)
			action_display = "turn right"

		goal_steer_modifier = .5
		if(distance_to_ball < 300):
			turn += clamp(o_goal_scr * goal_steer_modifier, -1.0, 1.0)
			
			if(abs(o_goal_scr) < 0.1 and ball_location.z < 150):
				desired_speed += 400

		self.controller_state.handbrake = abs(ball_scr) > 1
		self.controller_state.steer = turn
		self.controller_state.boost = (desired_speed - current_speed) > 200
		self.controller_state.throttle = clamp((desired_speed - current_speed) / 100.0, -1.0, 1.0)
		#draw_debug(self.renderer, my_car, packet.game_ball, action_display)

		return self.controller_state

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

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