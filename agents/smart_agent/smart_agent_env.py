import numpy as np
import random
import gymnasium as gym
import wrapper

class TerraBotEnvironment(gym.Env):
	# [weight, humidity, temperature, light level]
	# TODO find actual optimal weight
	def __init__(self, size=5):
		self._targets = np.array([400, 75, 26, 900])

		# SENSORS------------------------------------  ACTUATORS----------------------------
		# [weight, humidity, temperature, light level, led strength, fan on/off, pump on/off]
		self.observation_space = gym.spaces.Box(0, 1000, shape=(7,), dtype=int)

		# (255 - 0 + 1) + 2 + 2 = 256 + 4 + 260
		self.action_space = gym.spaces.Discrete(260)

		for i in range(256):
			self._action_to_actuator[i] = np.array([i, 0, 0])
		self._action_to_actuator[256] = [0, 0, 0]
		self._action_to_actuator[257] = [0, 1, 0]
		self._action_to_actuator[258] = [0, 0, 0]
		self._action_to_actuator[259] = [0, 0, 1]

	def _get_observations(self):
		return np.concatenate((self._sensors, self._targets, self._actuators))
	def _get_info(self):
		return np.linalg.norm(self._sensors - self._targets, ord=2)

	def reset(self, seed=None, options=None):
		super().reset(seed=seed)

		self._sensors = np.array([
			wrapper.get_weight(),
			wrapper.get_humidity(),
			wrapper.get_temperature(),
			wrapper.get_light_level()
		])
		self._targets = np.copy(self._sensors)
		while not np.array_equal(self._sensors, self._target):
			self._targets = np.array([
				random.SystemRandom().randint(0, 1000),
				random.SystemRandom().randint(0, 100),
				random.SystemRandom().randint(10, 40),
				random.SystemRandom().randint(0, 1000)
			])

		# self._actuators = np.array([
		# 	random.SystemRandom().randint(0, 255),
		# 	random.SystemRandom().randint(0, 1),
		# 	random.SystemRandom().randint(0, 1)
		# ])
		# self.targets = np.array([400, 75, 26, 900])

		return self._get_observations(), self._get_info()

	def step(self, action):
		self._sensors = np.array([
			wrapper.get_weight(),
			wrapper.get_humidity(),
			wrapper.get_temperature(),
			wrapper.get_light_level()
		])
		self._actuators = self._action_to_actuator[action]

		terminated =  np.array_equal(self._sensors, self._target)
		reward = 1 if terminated else 0

		return self._get_observations(), reward, terminated, False, self._get_info()
