import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    # Populate path time using desired velocity heuristic
    path = np.asarray(path)

    t_path = [0]
    total_t = 0

    for i in range(1, len(path)):
        t_step = np.linalg.norm(path[i,:] - path[i-1,:]) / V_des
        total_t += t_step
        t_path.append(total_t)

    x_tck = scipy.interpolate.splrep(t_path, path[:,0], k=3, s=alpha)
    y_tck = scipy.interpolate.splrep(t_path, path[:,1], k=3, s=alpha)

    t_smoothed = np.linspace(0, t_path[-1], int(t_path[-1] / dt))

    x = scipy.interpolate.splev(t_smoothed, x_tck)
    y = scipy.interpolate.splev(t_smoothed, y_tck)
    xd = scipy.interpolate.splev(t_smoothed, x_tck, der=1)
    yd = scipy.interpolate.splev(t_smoothed, y_tck, der=1)
    xdd = scipy.interpolate.splev(t_smoothed, x_tck, der=2)
    ydd = scipy.interpolate.splev(t_smoothed, y_tck, der=2)
    th = np.arctan2(yd, xd)

    traj_smoothed = np.vstack((x, y, th, xd, yd, xdd, ydd)).T
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
