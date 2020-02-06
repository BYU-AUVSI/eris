import numpy as np

def calc_obs_den(n_obs, obs, obs_rad, uav_ws):
    """
    Parameters
    ----------
    n_obs
        The number of obstacles
    obs
        The locations of the obstacles as a list of lists.
        [
            [x, y],
            [x, y],
            ...
        ]
    obs_rad
        The radii of the obstacles passed in to obs. Length of `obs` list
        and `obs_rad` should be the same, as it is a one-to-one mapping.
    uav_ws
        The wingspan of the UAV.
    """

    points = 1000

    x = np.linspace(-500,500,points)
    y = np.linspace(-500,500,points)

    field = np.zeros((points, points))

    for i in range(len(x)):
        for j in range(len(y)):
            for k in range(n_obs):
                if ((obs[k,0]-x[i])**2+(obs[k,1]-y[i])**2)**0.5 < (obs_rad[k]):
                    field[i,j] = 1

    obs_density = np.sum(np.sum(field))/points**2

    return obs_density