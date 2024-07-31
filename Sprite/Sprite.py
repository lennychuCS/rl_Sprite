import math
from enum import Enum

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3

# https://python-statemachine.readthedocs.io/en/latest/readme.html
opp_goal_locations = [Vec3(0, 5200, 0), Vec3(0, -5200, 0)]
my_goal_locations = [Vec3(0, -5200, 0), Vec3(0, 5200, 0)]

class State(Enum):
	KICKOFF = 0
	OFFENSE = 1
	DEFENSE = 2
	HELPER = 3
	SHOOTING = 4
	
class RLSprite(BaseAgent):

	def initialize_agent(self):
		global ball_goal_vels
		global ball_goal_vecs
		global connection_points
		global helper_points

		# This runs once before the bot starts up
		self.controller_state = SimpleControllerState()
		self.state = State.HELPER
		self.been_home = False
		target_loc = Vec3(0, 0, 0)

		# Vector to hit ball into goal
		ball_goal_vecs = [Vec3(0, 0, 0)] * 120

		# Velocity needed to hit ball at
		ball_goal_vels = [0] * 120

		# Where we predict we hit the ball
		connection_points = [Vec3(0, 0, 0)] * 120

		# Lineup helper for connection points
		helper_points = [Vec3(0, 0, 0)] * 120
	
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
		global car_facing_ball
		global ball_goal_vels
		global ball_goal_vecs
		global connection_points
		global helper_points

		# Get Packet Info at Start
		my_car = packet.game_cars[self.index]
		ball_loc = Vec3(packet.game_ball.physics.location)
		my_loc = Vec3(my_car.physics.location)
		current_speed = Vec3(my_car.physics.velocity).flat().length()

		self.ball_prediction = self.get_ball_prediction_struct()

		target_loc = Vec3(0, 0, 0)

		if self.ball_prediction:
			for i in range(120):
				ball_goal_vecs[i] = (opp_goal_locations[self.team] - Vec3(self.ball_prediction.slices[i].physics.location)).normalized()
				ball_goal_vels[i] = (ball_goal_vecs[i] * 300).dist(self.ball_prediction.slices[i].physics.velocity)
				connection_points[i] = Vec3(self.ball_prediction.slices[i].physics.location).flat()
				helper_points[i] = (Vec3(self.ball_prediction.slices[i].physics.location) - (ball_goal_vecs[i] * ball_goal_vels[i])).flat()

		if(self.state == State.DEFENSE):
			if(self.been_home):
				target_loc = connection_points[0]
			else:
				target_loc = Vec3(0, -4400, 50)
		elif(self.state == State.HELPER):
			target_loc = helper_points[0]
		elif(self.state == State.SHOOTING):
			target_loc = connection_points[0]

		target_loc = Vec3(clamp(target_loc.x, -4000, 4000), clamp(target_loc.y, -5100, 5100), 50)
		car_to_target = Vec3(target_loc) - Vec3(my_loc)
		dis_to_target = car_to_target.flat().length()
		time_to_target = 1000 if current_speed == 0 else dis_to_target / current_speed

		if(ball_loc.y < -2500):
			if(self.state != State.DEFENSE):
				self.state = State.DEFENSE
				self.been_home = False
			else:
				if(dis_to_target < 200):
					self.been_home = True
		elif(self.state == State.HELPER):
			if(my_loc.y < target_loc.y and dis_to_target < 100):
				self.state = State.SHOOTING
		elif(self.state == State.SHOOTING):
			if(my_loc.y > target_loc.y or dis_to_target > 2000 or (packet.game_ball.latest_touch.time_seconds <= 0.1 and packet.game_ball.latest_touch.player_index == self.index)):
				self.state = State.HELPER
		else:
			self.state = State.HELPER

		self.bot_wheels = my_car.has_wheel_contact
		self.bot_boost = my_car.boost
		self.bot_roll = my_car.physics.rotation.roll

		# Find the direction of our car using the Orientation class
		car_orientation = Orientation(my_car.physics.rotation)
		car_direction = car_orientation.forward

		desired_speed = clamp(ball_goal_vels[0] * 2, 800, 2000)

		# Steer Correction Radians
		target_scr = find_correction(car_direction, car_to_target)

		s_threash = 0.05

		if target_scr > (0 + s_threash):
			# Positive radians in the unit circle is a turn to the left.
			turn = -min(abs(target_scr), 1.0)  # Negative value for a turn to the left.
			action_display = "turn left"
		else:
			turn = min(abs(target_scr), 1.0)
			action_display = "turn right"

		self.controller_state.handbrake = abs(target_scr) > 1.5
		self.controller_state.boost = (desired_speed - current_speed) > 200
		self.controller_state.steer = turn
		self.controller_state.throttle = clamp((desired_speed - current_speed) / 100.0, -0.2, 1.0)
		draw_debug(self.renderer, my_car, target_loc, action_display, str(self.state))
		#draw_shots(self.renderer, connection_points, helper_points, 60)

		#if self.ball_prediction:
		#	draw_ball_path(self.renderer, self.ball_prediction)

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

def draw_shots(renderer, locs1, locs2, num):
	renderer.begin_rendering()
	for i in range(num):
		renderer.draw_line_3d(locs1[i], locs2[i], renderer.white())
	renderer.end_rendering()

def draw_debug(renderer, car, loc, action_display, distance):
	renderer.begin_rendering()
	# draw a line from the car to the ball
	renderer.draw_line_3d(car.physics.location, loc, renderer.white())
	# print the action that the bot is taking
	renderer.draw_string_3d(car.physics.location, 2, 2, distance, renderer.white())
	renderer.end_rendering()

def draw_ball_path(renderer, bp):
        poly=[]
        for i in range(bp.num_slices):
            poly.append(bp.slices[i].physics.location)
        renderer.begin_rendering()
        # draw a line from the car to the ball
        renderer.draw_polyline_3d(poly, renderer.white())
        renderer.end_rendering()