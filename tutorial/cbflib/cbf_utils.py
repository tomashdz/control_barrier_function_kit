from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np


def example(i):
    # Examples of different bad sets as ellipses
    # Defined as [z_0 ,z_1 , a, b] where z_0, z_1 represent the center and a, b represents the major, minor axes of the ellipse
    switcher = {
        0: [[3., 3., 1., 1]],
        1: [[1., 2., 0.5, 0.5], [4., 1., 0.5, 0.5],
            [3., 2., 0.5, 0.5], [4.5, 4.2, 0.5, 0.5]],
        2: [[3.5, 1., 0.2, 2.], [2., 2.5, 1., 0.2], [1.5, 1., 0.5, 0.5]],
        3: [[3.5, 3., 0.2, 2.], [2., 2.5, 1., 0.2], [1.5, 1., 0.5, 0.5]],
        4: [[10, 3.5, 2, 1]]
    }
    return switcher.get(i, "Invalid")


def is_inside_ellipse(x, ell_params):
    # Check if state is inside ellipse
    if ((x[0] - ell_params[0])/ell_params[2])**2 + ((x[1] - ell_params[1])/ell_params[3])**2 <= 1:
        return 1
    else:
        return 0


def plot_cbf_elements(ax, bad_sets, goal_x):
    # Plot the bad sets and the goal region
    for idxi, _ in enumerate(bad_sets):
        curr_bs = bad_sets[idxi]
        ell = Ellipse((curr_bs[0], curr_bs[1]), 2 *
                      curr_bs[2], 2 * curr_bs[3], color='r', alpha=0.3)
        ax.add_patch(ell)

    goal_square = plt.Rectangle(
        goal_x-np.array([.1, .1]), .2, .2, color='g', alpha=0.5)

    ax.add_patch(goal_square)

    return ax
