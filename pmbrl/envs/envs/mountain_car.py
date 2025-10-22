import math
import gym
import numpy as np
from typing import Optional
from gym.envs.classic_control.mountain_car import MountainCarEnv

class SparseMountainCarEnv(MountainCarEnv):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 30}

    def __init__(self, render_mode: Optional[str] = "rgb_array", goal_velocity=0):

        self.min_action = -1.0
        self.max_action = 1.0
        self.min_position = -1.2
        self.max_position = 0.6
        self.max_speed = 0.07
        self.goal_position = 0.45
        self.goal_velocity = goal_velocity
        self.power = 0.0015
        self.low_state = np.array([self.min_position, -self.max_speed])
        self.high_state = np.array([self.max_position, self.max_speed])
        self.render_mode = render_mode
        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True
        self.action_space = gym.spaces.Box(
            low=self.min_action, high=self.max_action, shape=(1,), dtype=np.float32
        )
        self.observation_space = gym.spaces.Box(
            low=self.low_state, high=self.high_state, dtype=np.float32
        )
        self.seed()
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def step(self, action):
        position = self.state[0]
        velocity = self.state[1]
        force = min(max(action[0], -1.0), 1.0)

        velocity += force * self.power - 0.0025 * math.cos(3 * position)
        if velocity > self.max_speed:
            velocity = self.max_speed
        if velocity < -self.max_speed:
            velocity = -self.max_speed
        position += velocity
        if position > self.max_position:
            position = self.max_position
        if position < self.min_position:
            position = self.min_position
        if position == self.min_position and velocity < 0:
            velocity = 0

        done = bool(position >= self.goal_position and velocity >= self.goal_velocity)

        reward = 0
        if done:
            reward = 1.0

        self.state = np.array([position, velocity])
        return self.state, reward, done, {}

    def reset(self):
        self.state = np.array([self.np_random.uniform(low=-0.6, high=-0.4), 0])
        return np.array(self.state)

    # def render(self, mode="rgb_array"):
    #     try:
    #         import pygame
    #     except ImportError:
    #         raise ImportError("pygame is not installed, run `pip install pygame`")

    #     screen_width = 600
    #     screen_height = 400
    #     world_width = self.max_position - self.min_position
    #     scale = screen_width / world_width
    #     carwidth = 40
    #     carheight = 20

    #     if self.screen is None:
    #         pygame.init()
    #         if mode == "human":
    #             pygame.display.init()
    #             self.screen = pygame.display.set_mode((screen_width, screen_height))
    #         else:
    #             self.screen = pygame.Surface((screen_width, screen_height))
    #     if self.clock is None:
    #         self.clock = pygame.time.Clock()

    #     self.surf = pygame.Surface((screen_width, screen_height))
    #     self.surf.fill((255, 255, 255))

    #     # 绘制山
    #     xs = np.linspace(self.min_position, self.max_position, 100)
    #     ys = self._height(xs)
    #     xys = []
    #     for x, y in zip(xs, ys):
    #         px = (x - self.min_position) * scale
    #         py = screen_height - y
    #         xys.append((int(np.clip(px, 0, screen_width)), int(np.clip(py, 0, screen_height))))
    #     pygame.draw.aalines(self.surf, (0, 0, 0), False, xys, 4)

    #     # 绘制车
    #     pos = self.state[0]
    #     clearance = 10
        
    #     cart_x = (pos - self.min_position) * scale
    #     cart_y = screen_height - self._height(pos) - clearance
        
    #     # 车身坐标
    #     cart_coords = [
    #         (cart_x - carwidth / 2, cart_y),
    #         (cart_x - carwidth / 2, cart_y - carheight),
    #         (cart_x + carwidth / 2, cart_y - carheight),
    #         (cart_x + carwidth / 2, cart_y)
    #     ]
    #     cart_coords = [(int(x), int(y)) for x, y in cart_coords]
    #     pygame.draw.polygon(self.surf, (0, 0, 0), cart_coords)

    #     # 绘制轮子
    #     wheel_radius = int(carheight / 2.5)
    #     wheel1_x = int(np.clip(cart_x + carwidth / 4, wheel_radius, screen_width - wheel_radius))
    #     wheel1_y = int(np.clip(cart_y, wheel_radius, screen_height - wheel_radius))
    #     wheel2_x = int(np.clip(cart_x - carwidth / 4, wheel_radius, screen_width - wheel_radius))
    #     wheel2_y = int(np.clip(cart_y, wheel_radius, screen_height - wheel_radius))
        
    #     pygame.draw.circle(self.surf, (128, 128, 128), (wheel1_x, wheel1_y), wheel_radius)
    #     pygame.draw.circle(self.surf, (128, 128, 128), (wheel2_x, wheel2_y), wheel_radius)

    #     # 绘制目标旗帜
    #     flagx = int((self.goal_position - self.min_position) * scale)
    #     flagy = int(screen_height - self._height(self.goal_position))
    #     pygame.draw.line(self.surf, (0, 0, 0), (flagx, flagy), (flagx, max(0, flagy - 50)), 2)
    #     flag_coords = [(flagx, max(0, flagy - 50)), (flagx, max(0, flagy - 40)), (flagx + 25, max(0, flagy - 45))]
    #     pygame.draw.polygon(self.surf, (204, 204, 0), flag_coords)

    #     self.surf = pygame.transform.flip(self.surf, False, True)
    #     self.screen.blit(self.surf, (0, 0))

    #     if mode == "human":
    #         pygame.event.pump()
    #         self.clock.tick(self.metadata["video.frames_per_second"])
    #         pygame.display.flip()
        
    #     if mode == "rgb_array":
    #         return np.transpose(np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2))
        
    #     return None

    # def _height(self, xs):
    #     return np.sin(3 * xs) * 0.45 * 400 + 0.55 * 400

    # def close(self):
    #     if self.screen is not None:
    #         import pygame
    #         pygame.display.quit()
    #         pygame.quit()
    #         self.screen = None
    #         self.clock = None
