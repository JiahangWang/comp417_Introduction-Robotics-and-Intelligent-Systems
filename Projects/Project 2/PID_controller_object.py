import numpy as np

class PID_controller:
    def __init__(self, Kp=2, Ki=0.001, Kd=1, max_integral=1):
        # Initialize PID controller parameters
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.max_integral = max_integral  # Maximum integral term value to prevent integral windup
        self.prev_action = 0  # Previous action value in torque
        self.integral_control = True

        # Controller state variables
        self.integral = 0  # Integral term
        self.prev_error = 0  # Previous error

    def reset_state(self):
        # Reset the controller state
        self.integral = 0
        self.prev_error = 0

    def get_action(self, state, image_state, random_controller=False):
        # Extract state information
        terminal, timestep, x, x_dot, theta, theta_dot, reward = state

        if random_controller:
            # Return a random action if random control is applied
            return np.random.uniform(-1, 1)
        else:
            # Calculate the current error
            error = theta  # The goal is to maintain the pole upright, i.e., theta is 0

            # Update the integral term
            self.integral += error

            # Calculate the error derivative
            derivative = (error - self.prev_error)

            # Anti-windup measure for the integral term
            if self.integral_control:
                self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)

            # Compute the PID controller output
            action = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

            if np.random.rand() > 0.99:
                action = 10  # Strong disturbance to simulate sudden events

            # Save the current error for the next iteration
            self.prev_error = error

            return action
