#!/usr/bin/env python
from __future__ import print_function
import matplotlib.pylab as plt
import os

from .builder import create_trajectory

def new_line(f):
    def wrapper(*args, **kwargs):
        print()
        f(*args, **kwargs)
        print()
    return wrapper


def plot_trajectory(name):
    STEPS = 600
    DELTA = 1 if name != 'linear' else 0.1
    trajectory = create_trajectory(name, STEPS)

    x = [trajectory.get_position_at(i * DELTA).x for i in range(STEPS)]
    y = [trajectory.get_position_at(i * DELTA).y for i in range(STEPS)]

    trajectory_fig, trajectory_plot = plt.subplots(1, 1)
    trajectory_plot.plot(x, y, label='trajectory', lw=3)
    trajectory_plot.set_title(name.title() + ' Trajectory', fontsize=20)
    trajectory_plot.set_xlabel(r'$x{\rm[m]}$', fontsize=18)
    trajectory_plot.set_ylabel(r'$y{\rm[m]}$', fontsize=18)
    trajectory_plot.legend(loc=0)
    trajectory_plot.grid()
    plt.show()


class TerminateProgramException(Exception):
    pass


class Menu:
    MENU = """Menu:
1. List Trajectories
2. Plot Trajectory
3. Quit
        """

    def __init__(self):
        self.trajectories = self._get_trajectories()
        self.choices = {
            '1': self.list_trajectories,
            '2': self.plot_trajectory,
            '3': self.quit
        }

    @staticmethod
    def _get_trajectories():
        path = os.sep.join(__file__.split(os.sep)[:-1])
        trajectories = [file.replace('_trajectory.py', '')
                        for file in os.listdir(path)
                        if '_trajectory' in file and '.pyc' not in file]

        return {str(i + 1): trajectory
                for i, trajectory in enumerate(trajectories)}

    def run(self):
        while True:
            print(self.MENU)
            choice = raw_input('Enter an option: ')

            try:
                self.choices[choice]()
            except KeyError:
                print('Error: "{}" is not a valid option.'.format(choice))
            except TerminateProgramException:
                break

    @new_line
    def list_trajectories(self):
        print('Trajectories:')
        for k, v in sorted(self.trajectories.items()):
            print('{}. {}'.format(k, v))

    @new_line
    def plot_trajectory(self):
        while True:
            self.list_trajectories()
            choice = raw_input('What trajectory do you want plot? ')
            try:
                trajectory = self.trajectories[choice]
            except KeyError:
                print('Invalid option.')
            else:
                plot_trajectory(trajectory)
                break

    def quit(self):
        raise TerminateProgramException()