import numpy as np
import random
import gymnasium as gym
import wrapper

class TerraBotEnvironment(gym.Env):
	# [weight, humidity, temperature, light level]
	# TODO find actual optimal weight
	def __init__(self, size=5):
		self._targets = np.array([400, 75, 26, 900])

		# sensors, targets, actuators
		self.observation_space = gym.spaces.Box(0, 1000, shape=(11,), dtype=int)

		# (255 - 0 + 1) + 2 + 2 = 256 + 4 + 260
		# self.action_space = gym.spaces.Discrete(8)
		self.action_space = gym.spaces.Discrete(4)

		self._action_to_actuators = [
			[0, 0, 0],
			#[0, 0, 1],
			[0, 1, 0],
			#[0, 1, 1],
			[255, 0, 0],
			#[255, 0, 1],
			[255, 1, 0],
			#[255, 1, 1]
		]

	def _get_observations(self):
		return np.concatenate((self._sensors, self._targets, self._actuators))
	def _get_info(self):
		return np.linalg.norm(self._sensors - self._targets, ord=1)
	def _apply_actuators(self):
		wrapper.set_led(self._actuators[0])
		wrapper.set_fan(self._actuators[1])
		wrapper.set_pump(0)
		# wrapper.set_pump(self._actuators[2])

	def reset(self, seed=None, options=None):
		super().reset(seed=seed)

		self._sensors = np.array([
			wrapper.get_weight(),
			wrapper.get_humidity(),
			wrapper.get_temperature(),
			wrapper.get_light_level()
		])
		self._targets = np.copy(self._sensors)
			while not np.array_equal(self._sensors, self._targets):
			self._targets = np.array([
				random.SystemRandom().randint(0, 1000),
				random.SystemRandom().randint(0, 100),
				random.SystemRandom().randint(10, 40),
				random.SystemRandom().randint(0, 1000)
			])

			print(self._sensors)
		print(self._targets)

		self._actuators = np.array([0, 0, 0])
		self._apply_actuators()

		return self._get_observations(), self._get_info()

	def step(self, action):
		self._sensors = np.array([
			wrapper.get_weight(),
			wrapper.get_humidity(),
			wrapper.get_temperature(),
			wrapper.get_light_level()
		])
		self._actuators = self._action_to_actuators[action]
		self._apply_actuators()

		reward = self._get_info()
		terminated = reward <= 1
		reward = 1e9 if reward == 0 else abs(3 / reward)

		return self._get_observations(), reward, terminated
	
