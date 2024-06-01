import math
import random
import numpy as np
import matplotlib.pyplot as plt

class Waypoint_Generator:
    def oval(self, a=5, b=3, center=(0, 0), num_points=25):
        """Generates waypoints for an oval trajectory.

        Args:
            a: Semi-major axis of the oval.
            b: Semi-minor axis of the oval.
            center: Center of the oval.
            num_points: Number of waypoints to generate.

        Returns:
            A list of waypoints.
        """
        theta = np.linspace(0, 2*np.pi, num_points)
        x = center[0] + a * np.cos(theta)
        y = center[1] + b * np.sin(theta)
        waypoints = np.column_stack((x, y))
        return waypoints + self.initial_position

    def half_moon(self, radius=5, center=(0, 0), num_points=15, width=2):
        """Generates waypoints for a half-moon trajectory.

        Args:
            radius: Radius of the half-moon.
            center: Center of the half-moon.
            num_points: Number of waypoints to generate.

        Returns:
            A list of waypoints.
        """
        theta = np.linspace(0, np.pi, num_points)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        
        # Adjust for the opening in the half-moon
        y = y + width * np.sin(2 * theta)

        waypoints = np.column_stack((x, y))

        return waypoints + self.initial_position
    
    def sin_wave(self, amplitude=5, frequency=1.5, phase=0, num_points=25):
        """Generates waypoints for a sinusoidal trajectory.

        Args:
            amplitude: Amplitude of the sine wave.
            frequency: Frequency of the sine wave.
            phase: Phase shift of the sine wave.
            num_points: Number of waypoints to generate.

        Returns:
            A list of waypoints.
        """
        theta = np.linspace(0, 2*np.pi, num_points)
        x = theta
        y = amplitude * np.sin(frequency * theta + phase)
        waypoints = np.column_stack((x, y))
        return waypoints + self.initial_position

    def star(self, num_points=25, radius=5, center=(0, 0)):
        """Generates waypoints for a star trajectory.

        Args:
            num_points: Number of points in the star.
            radius: Radius of the star.
            center: Center of the star.

        Returns:
            A list of waypoints.
        """
        theta = np.linspace(0, 2*np.pi, num_points*2, endpoint=False)
        x = center[0] + radius * np.cos(theta) * (1 + 0.5 * np.cos(5 * theta))
        y = center[1] + radius * np.sin(theta) * (1 + 0.5 * np.cos(5 * theta))
        waypoints = np.column_stack((x, y))
        return waypoints + self.initial_position

    def figure_8(self, a: float = math.sqrt(4), center=(0, 0), num_points=30):
        """Generates waypoints for a figure-8 trajectory.

        Args:
            a: Semi-major axis of the figure-8.
            b: Semi-minor axis of the figure-8.
            center: Center of the figure-8.
            num_points: Number of waypoints to generate.

        Returns:
            A list of waypoints.
        """
        theta = np.linspace(-math.pi / 2, 3 * math.pi / 2, num_points)
        x = center[0] + a * np.cos(theta) / (np.sin(theta) ** 2 + 1) * 2
        y = center[1] + a * np.cos(theta) * np.sin(theta) / (np.sin(theta) ** 2 + 1)
        waypoints = np.column_stack((x, y))
        return waypoints + self.initial_position

    def square(self, side_length=10, center=(0, 0)):
        """Generates waypoints for a square trajectory.

        Args:
            side_length: Length of the side of the square.
            center: Center of the square.

        Returns:
            A list of waypoints.
        """
        half_side = side_length / 2
        waypoints = np.array([
            [center[0] - half_side, center[1] - half_side],
            [center[0] + half_side, center[1] - half_side],
            [center[0] + half_side, center[1] + half_side],
            [center[0] - half_side, center[1] + half_side],
            [center[0] - half_side, center[1] - half_side]  # Close the loop
        ])
        return waypoints + self.initial_position
    
    def choose_random_trajectory(self, initial_position):
        """Chooses a trajectory at random from the available functions.

        Returns:
            A NumPy array of waypoints for the randomly chosen trajectory.
        """
        self.initial_position = initial_position
        trajectory_functions = [self.oval, self.half_moon, self.sin_wave, self.star, self.figure_8, self.square]
        print(self.initial_position)
        random_function = random.choice(trajectory_functions)
        return random_function()
    

def visualize_waypoints(waypoints):
    """Visualizes a set of waypoints using matplotlib.

    Args:
        waypoints: A NumPy array of waypoints, where each row is a [x, y] coordinate.
    """
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    plt.figure(figsize=(8, 6))  # Adjust figure size as needed
    plt.plot(x, y, 'bo-')  # Plot waypoints as blue circles connected by lines
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Waypoints Visualization")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":

    # Example usage:
    waypoint_generator = Waypoint_Generator()

    # Generate waypoints for a randomly chosen trajectory
    random_waypoints = waypoint_generator.choose_random_trajectory([-1,2])
    print(random_waypoints)

    # Visualize the waypoints
    visualize_waypoints(random_waypoints)
