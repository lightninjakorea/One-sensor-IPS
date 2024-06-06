import numpy as np
import matplotlib.pyplot as plt


class ParticleFilter:
    def __init__(self, num_particles, x_range, y_range, process_variance, measurement_variance):
        self.num_particles = num_particles
        self.particles = np.empty((num_particles, 2))
        self.particles[:, 0] = np.random.uniform(x_range[0], x_range[1], num_particles)
        self.particles[:, 1] = np.random.uniform(y_range[0], y_range[1], num_particles)
        self.weights = np.ones(num_particles) / num_particles
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def predict(self, movement):
        noise = np.random.normal(0, self.process_variance, (self.num_particles, 2))
        self.particles += movement + noise

    def update(self, measurement):
        diff = self.particles - measurement
        distance = np.linalg.norm(diff, axis=1)
        self.weights = np.exp(-distance ** 2 / (2 * self.measurement_variance))
        self.weights += 1.e-300  # prevent divide by zero
        self.weights /= np.sum(self.weights)

    def resample(self):
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)