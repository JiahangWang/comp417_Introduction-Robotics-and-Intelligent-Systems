import pygame, sys
import numpy as np
from pygame.locals import *
import math
import PID_controller_object
import argparse
import time
import pygame.camera
from PIL import Image
import matplotlib.pyplot as plt


# Base engine from the following link
# https://github.com/the-lagrangian/inverted-pendulum

class InvertedPendulum(object):
    def __init__(self, windowdims, cartdims, penddims, gravity, add_noise_to_gravity_and_mass, action_range=[-1, 1]):

        self.action_range = action_range

        self.window_width = windowdims[0]
        self.window_height = windowdims[1]

        self.cart_width = cartdims[0]
        self.car_height = cartdims[1]
        self.pendulum_width = penddims[0]
        self.pendulum_length = penddims[1]

        self.Y_CART = 3 * self.window_height / 4
        self.reset_state()
        if add_noise_to_gravity_and_mass:
            self.gravity = gravity + np.random.uniform(-5, 5)
            self.masscart = 1.0 + np.random.uniform(-0.5, 0.5)
            self.masspole = 0.1 + np.random.uniform(-0.05, 0.2)
        else:
            self.gravity = gravity
            self.masscart = 1.0
            self.masspole = 0.1
        self.total_mass = self.masspole + self.masscart
        self.length = 0.5  # actually half the pole's length
        self.polemass_length = self.masspole * self.length
        self.force_mag = 10.0
        self.dt = 0.005  # seconds between state updates
        # Angle at which to fail the episode
        self.theta_threshold_radians = 180 * math.pi / 360
        self.x_threshold = 2.4

        self.x_conversion = self.window_width / 2 / self.x_threshold

    def reset_state(self):
        """initializes pendulum in upright state with small perturbation"""
        self.terminal = False
        self.timestep = 0

        self.x_dot = np.random.uniform(-0.03, 0.03)
        self.x = np.random.uniform(-0.01, 0.01)

        self.theta = np.random.uniform(-0.03, 0.03)
        self.theta_dot = np.random.uniform(-0.01, 0.01)
        self.score = 0
        self.reward = 0

    def get_state(self):
        return (self.terminal, self.timestep, self.x,
                self.x_dot, self.theta, self.theta_dot, self.reward)

    def set_state(self, state):
        terminal, timestep, x, x_dot, theta, theta_dot = state
        self.terminal = terminal
        self.timestep = timestep
        self.x = x_dot
        self.x_dot = x_dot
        self.theta = theta  # in radians
        self.theta_dot = theta_dot  # in radians

    def step(self, action):

        # From OpenAI CartPole
        # https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py
        self.timestep += 1
        action = np.clip(action, -1, 1)  # Max action -1, 1
        force = action * 10  # multiply action by 10 to scale
        costheta = math.cos(self.theta)
        sintheta = math.sin(self.theta)
        # For the interested reader:
        # https://coneural.org/florian/papers/05_cart_pole.pdf
        temp = (
                       force + self.polemass_length * self.theta_dot ** 2 * sintheta
               ) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (
                self.length * (4.0 / 3.0 - self.masspole * costheta ** 2 / self.total_mass)
        )
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        self.x = self.x + self.dt * self.x_dot
        self.x_dot = self.x_dot + self.dt * xacc
        self.theta = self.theta + self.dt * self.theta_dot
        self.theta_dot = self.theta_dot + self.dt * thetaacc

        self.terminal = bool(
            self.x < -self.x_threshold
            or self.x > self.x_threshold
            or self.theta < -self.theta_threshold_radians
            or self.theta > self.theta_threshold_radians
        )
        # radians to degrees
        # within -+ 15
        if (self.theta * 57.2958) < 15 and (self.theta * 57.2958) > -15:
            self.score += 1
            self.reward = 1
        else:
            self.reward = 0

        return self.get_state()


class InvertedPendulumGame(object):
    def __init__(self, figure_path, windowdims=(800, 400), cartdims=(50, 10), penddims=(6.0, 150.0), refreshfreq=1000,
                 gravity=9.81, manual_action_magnitude=1,
                 random_controller=False, max_timestep=1000, noisy_actions=False, mode=None,
                 add_noise_to_gravity_and_mass=False):

        self.PID_controller = mode
        self.max_timestep = max_timestep
        self.pendulum = InvertedPendulum(windowdims, cartdims, penddims, gravity, add_noise_to_gravity_and_mass)
        self.performance_figure_path = figure_path

        self.window_width = windowdims[0]
        self.window_height = windowdims[1]

        self.cart_width = cartdims[0]
        self.car_height = cartdims[1]
        self.pendulum_width = penddims[0]
        self.pendulum_length = penddims[1]
        self.manual_action_magnitude = manual_action_magnitude
        self.random_controller = random_controller
        self.noisy_actions = noisy_actions

        self.score_list = []

        self.Y_CART = self.pendulum.Y_CART
        # self.time gives time in frames
        self.timestep = 0

        pygame.init()
        self.clock = pygame.time.Clock()
        # specify number of frames / state updates per second
        self.REFRESHFREQ = refreshfreq
        self.surface = pygame.display.set_mode(windowdims, 0, 32)
        pygame.display.set_caption('Inverted Pendulum Game')
        # array specifying corners of pendulum to be drawn
        self.static_pendulum_array = np.array(
            [[-self.pendulum_width / 2, 0],
             [self.pendulum_width / 2, 0],
             [self.pendulum_width / 2, -self.pendulum_length],
             [-self.pendulum_width / 2, -self.pendulum_length]]).T
        self.BLACK = (0, 0, 0)
        self.BLUE = (0, 0, 255)
        self.RED = (255, 0, 0)
        self.WHITE = (255, 255, 255)

    def draw_cart(self, x, theta):
        cart = pygame.Rect(
            self.pendulum.x * self.pendulum.x_conversion + self.pendulum.window_width / 2 - self.cart_width // 2,
            self.Y_CART, self.cart_width, self.car_height)
        pygame.draw.rect(self.surface, self.BLUE, cart)
        pendulum_array = np.dot(self.rotation_matrix(-theta), self.static_pendulum_array)
        pendulum_array += np.array([[x * self.pendulum.x_conversion + self.pendulum.window_width / 2], [self.Y_CART]])
        pendulum = pygame.draw.polygon(self.surface, self.RED,
                                       ((pendulum_array[0, 0], pendulum_array[1, 0]),
                                        (pendulum_array[0, 1], pendulum_array[1, 1]),
                                        (pendulum_array[0, 2], pendulum_array[1, 2]),
                                        (pendulum_array[0, 3], pendulum_array[1, 3])))

    @staticmethod
    def rotation_matrix(theta):
        return np.array([[np.cos(theta), np.sin(theta)],
                         [-1 * np.sin(theta), np.cos(theta)]])

    def render_text(self, text, point, position="center", fontsize=48):
        font = pygame.font.SysFont(None, fontsize)
        text_render = font.render(text, True, self.BLACK, self.WHITE)
        text_rect = text_render.get_rect()
        if position == "center":
            text_rect.center = point
        elif position == "topleft":
            text_rect.topleft = point
        self.surface.blit(text_render, text_rect)

    def time_seconds(self):
        return self.timestep / float(self.REFRESHFREQ)

    def starting_page(self):
        self.surface.fill(self.WHITE)
        self.render_text("Inverted Pendulum",
                         (0.5 * self.window_width, 0.4 * self.window_height))
        self.render_text("COMP 417 Assignment 2",
                         (0.5 * self.window_width, 0.5 * self.window_height),
                         fontsize=30)
        self.render_text("Press Enter to Begin",
                         (0.5 * self.window_width, 0.7 * self.window_height),
                         fontsize=30)
        pygame.display.update()

    def save_current_state_as_image(self, path):
        im = Image.fromarray(self.surface_array)
        im.save(path + "current_state.png")

    def game_round(self):
        self.pendulum.reset_state()
        # if you need to reset sth add it here.

        ##########
        theta_diff_list = []

        action = 0
        for i in range(self.max_timestep):
            self.surface_array = pygame.surfarray.array3d(self.surface)
            self.surface_array = np.transpose(self.surface_array, [1, 0, 2])
            if self.PID_controller is None:
                for event in pygame.event.get():
                    if event.type == QUIT:
                        pygame.quit()
                        sys.exit()
                    if event.type == KEYDOWN:
                        if event.key == K_LEFT:
                            action = -self.manual_action_magnitude  # "Left"
                        if event.key == K_RIGHT:
                            action = self.manual_action_magnitude
                    if event.type == KEYUP:
                        if event.key == K_LEFT:
                            action = 0
                        if event.key == K_RIGHT:
                            action = 0
                        if event.key == K_ESCAPE:
                            pygame.quit()
                            sys.exit()
            else:
                action = self.PID_controller.get_action(self.pendulum.get_state(), self.surface_array,
                                                        random_controller=self.random_controller)
                for event in pygame.event.get():
                    if event.type == QUIT:
                        pygame.quit()
                        sys.exit()
                    if event.type == KEYDOWN:
                        if event.key == K_ESCAPE:
                            print("Exiting ... ")
                            pygame.quit()
                            sys.exit()

            if self.noisy_actions and PID_controller_object is None:
                action = action + np.random.uniform(-0.1, 0.1)

            terminal, timestep, x, _, theta, _, _ = self.pendulum.step(action)
            theta_diff_list.append(np.abs(theta))

            self.timestep = timestep
            self.surface.fill(self.WHITE)
            self.draw_cart(x, theta)

            time_text = "t = {}".format(self.pendulum.score)
            self.render_text(time_text, (0.1 * self.window_width, 0.1 * self.window_height),
                             position="topleft", fontsize=40)

            pygame.display.update()
            self.clock.tick(self.REFRESHFREQ)
            if terminal or i == self.max_timestep - 1: # modified by Jiahang
                plt.plot(np.arange(len(theta_diff_list)), theta_diff_list)
                plt.xlabel('Time')
                plt.ylabel('Theta(radians)')
                plt.title("Theta vs Time")
                plt.grid()
                plt.savefig(self.performance_figure_path + "_run_" + str(len(self.score_list)) + ".png")
                plt.close()
                break;
        self.score_list.append(self.pendulum.score)

    def end_of_round(self):
        self.surface.fill(self.WHITE)
        self.draw_cart(self.pendulum.x, self.pendulum.theta)
        self.render_text("Score: {}".format(self.pendulum.score),
                         (0.5 * self.window_width, 0.3 * self.window_height))
        self.render_text("Average Score : {}".format(np.around(np.mean(self.score_list), 3)),
                         (0.5 * self.window_width, 0.4 * self.window_height))
        self.render_text("Standard Deviation Score : {}".format(np.around(np.std(self.score_list), 3)),
                         (0.5 * self.window_width, 0.5 * self.window_height))
        self.render_text("Runs : {}".format(len(self.score_list)),
                         (0.5 * self.window_width, 0.6 * self.window_height))
        if self.PID_controller is None:
            self.render_text("(Enter to play again, ESC to exit)",
                             (0.5 * self.window_width, 0.85 * self.window_height), fontsize=30)
        pygame.display.update()
        time.sleep(2.0)

    def game(self):
        self.starting_page()
        while True:
            if self.PID_controller is None:  # Manual mode engaged
                for event in pygame.event.get():
                    if event.type == QUIT:
                        pygame.quit()
                        sys.exit()
                    if event.type == KEYDOWN:
                        if event.key == K_RETURN:
                            self.game_round()
                            self.end_of_round()
                        if event.key == K_ESCAPE:
                            pygame.quit()
                            sys.exit()
            else:  # Use the PID controller instead, ignores input expect exit
                self.game_round()
                self.end_of_round()
                self.pendulum.reset_state()


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default="pid")
    parser.add_argument('--random_controller', type=bool, default=False)
    parser.add_argument('--add_noise_to_gravity_and_mass', type=bool, default=True)
    parser.add_argument('--max_timestep', type=int, default=2000)

    parser.add_argument('--gravity', type=float, default=9.81)
    # parser.add_argument('--gravity', type=float, default=15)
    parser.add_argument('--manual_action_magnitude', type=float, default=1)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--noisy_actions', type=bool, default=False)
    parser.add_argument('--performance_figure_path', type=str, default="performance_figure")

    args = parser.parse_args()
    return args


def main():
    args = get_args()
    np.random.seed(args.seed)
    if args.mode == "manual":
        inv = InvertedPendulumGame(args.performance_figure_path, mode=None, gravity=args.gravity,
                                   manual_action_magnitude=args.manual_action_magnitude,
                                   random_controller=args.random_controller,
                                   max_timestep=args.max_timestep, noisy_actions=args.noisy_actions,
                                   add_noise_to_gravity_and_mass=args.add_noise_to_gravity_and_mass)
    else:
        inv = InvertedPendulumGame(args.performance_figure_path, mode=PID_controller_object.PID_controller(),
                                   gravity=args.gravity, manual_action_magnitude=args.manual_action_magnitude,
                                   random_controller=args.random_controller, max_timestep=args.max_timestep,
                                   noisy_actions=args.noisy_actions,
                                   add_noise_to_gravity_and_mass=args.add_noise_to_gravity_and_mass)
    inv.game()


if __name__ == '__main__':
    main()
