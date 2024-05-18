import numpy as np
import random
import gymnasium as gym
import wrapper

class TerraBotEnvironment(gym.Env):
    # [weight, humidity, temperature, light level]
    # TODO find actual optimal weight
    def __init__(self, size=5):
        self._targets = np.array([75, 26, 900])

        # sensors, targets, actuators
        self.observation_space = gym.spaces.Box(0, 1000, shape=(8,), dtype=int)
        self.action_space = gym.spaces.Discrete(256 * 2)
        
        self.action_to_actuators = []
        for i in range(256):
            self.action_to_actuators.append([i, 0])
            self.action_to_actuators.append([i, 1])

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
            # wrapper.get_weight(),
            wrapper.get_humidity(),
            wrapper.get_temperature(),
            wrapper.get_light_level()
        ])
        self._targets = np.copy(self._sensors)
        while np.array_equal(self._sensors, self._targets):
            self._targets = np.array([
                # random.SystemRandom().randint(0, 1000),
                random.SystemRandom().randint(70, 80),
                random.SystemRandom().randint(25, 27),
                random.SystemRandom().randint(860, 940)
            ])

        print(self._sensors)
        print(self._targets)

        self._actuators = np.array([0, 0])
        self._apply_actuators()

        reward = self._get_info()
        reward = 1e9 if reward == 0 else abs(3 / reward)

        return self._get_observations(), reward

    def step(self, action):
        self._sensors = np.array([
            wrapper.get_humidity_raw()[1],
            wrapper.get_temperature_raw()[1],
            wrapper.get_light_level_raw()[1]
        ])
        self._actuators = self.action_to_actuators[action]
        self._apply_actuators()

        reward = self._get_info()
        terminated = reward <= 1
        reward = 1e9 if reward == 0 else abs(3 / reward)
        
        print("action: " + str(self.action_to_actuators[action]))
        print("sensor: " + str(self._sensors))
        print("target: " + str(self._targets))
        print("reward: " + str(reward))
        print("done:   " + str(terminated))

        return self._get_observations(), reward, terminated
    
