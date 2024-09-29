import numpy as np

class RL_controller:
    def __init__(self, args):
        self.gamma = args.gamma  # Discount factor
        self.lr = args.lr  # Learning rate
        self.Q_value = np.zeros((args.theta_discrete_steps, args.theta_dot_discrete_steps, 3))  # State-action value
        self.epsilon = 0.8  # Initial exploration rate
        self.epsilon_decay = 0.99  # Exploration rate decay
        self.epsilon_min = 0.1  # Minimum exploration rate
        self.epsilon_decay_step = 50  # Steps after which to start reducing exploration rate
        self.step_count = 0  # Step counter
        self.prev_s = None  # Previous state
        self.prev_a = None  # Previous action

    def reset(self):
        #You need to reset sth
        self.prev_s = None
        self.prev_a = None

    def get_action(self, state, image_state, random_controller=False, episode=0):
        terminal, timestep, theta, theta_dot, reward = state

        # Update step counter
        self.step_count += 1

        # if random_controller:
        #     action = np.random.randint(0, 3)  # you have three possible actions (0,1,2)

        # Select action
        if np.random.rand() < self.epsilon:
            action = np.random.randint(0, 3)  # # you have three possible actions (0,1,2)
        else:
            # use Q values to take the best action at each state
            action = np.argmax(self.Q_value[theta, theta_dot])

        # Update Q value
        if self.prev_s is not None and self.prev_a is not None:
            old_value = self.Q_value[self.prev_s[0], self.prev_s[1], self.prev_a]
            next_max = np.max(self.Q_value[theta, theta_dot])
            new_value = (1 - self.lr) * old_value + self.lr * (reward + self.gamma * next_max)
            self.Q_value[self.prev_s[0], self.prev_s[1], self.prev_a] = new_value

        # Reduce exploration rate
        if self.step_count > self.epsilon_decay_step:
            self.epsilon = max(self.epsilon * self.epsilon_decay, self.epsilon_min)

        # Save the current state and action
        self.prev_s = [theta, theta_dot]
        self.prev_a = action

        return action

    def get_state_value_matrix(self):
        state_value_matrix = np.max(self.Q_value, axis=2)
        return state_value_matrix