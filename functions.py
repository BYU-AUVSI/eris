import numpy as np

from glo import GLOBALS

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


def multi_start(ms_i):
    """
    Multi start functionality for optimization.

    Returns
    -------
    x_guess : 
    """

    su = 0

    #global variables
    num_path, x0, step_max, step_min, x_next, start = (
        GLOBALS.num_path,
        GLOBALS.x0,
        GLOBALS.step_max,
        GLOBALS.step_min,
        GLOBALS.x_next,
        GLOBALS.start
    )

    x_guess = np.zeros((2*num_path,2,ms_i))

    for i in range(ms_i):
        for j in range(2*num_path):
            #different starting paths for different cases
            if i == 0: #top right diagonal
                if num_path > 1:
                    if start == 1: #
                        if j >= 2*num_path-2:
                            x_guess[j,:,i] = [x_guess[2*num_path-2,0,i] + 0.25*(j-(2*num_path-2))*step_max, x_guess[2*num_path-2,1,1] + 0.25*(j-(2*num_path-2))*step_max + su]
                        else:
                            x_guess[j,:,i] = [x_next(j+2,0),x_next(j+2,1) + su]
                    else:
                        x_guess[j,:,i] = [x0[0] + 0.25*(j+1)*step_max, x0[1] + 0.25*(j+1)*step_max + su]
                else:
                    x_guess[j,:,i] = [x0[0] + 0.25*(j+1)*step_max, x0[1] + 0.25*(j+1)*step_max + su]

            elif i == 1: #right straight
                x_guess[j,:,i] = [x0[0] + 0.25*(j+1)*step_max, x0[1] + 0.25*((j+1)-(j+1)**2./(num_path*4.))*step_max]
                
            elif i == 2: #top straight
                x_guess[j,:,i] = [x0[0] + 0.25*((j+1)-(j+1)**2./(num_path*4.))*step_max, x0[1] + 0.25*(j+1)*step_max]       
                
            # FIXME: From here down, may need to change (j)'s to (j+1)
            elif i == 3: #to the right, slightly down
                x_guess[j,:,i] = [x0[0]+0.25*j*step_max, x0[1]-0.125*j*step_max]
                #x_guess(j,:,i) = [x0(1) + 0.125*j*step_max, x0(2) + 0.125*(j-j**2/(num_path*4))*step_max]
                
            elif i == 4: #to the top, slightly left
                x_guess[j,:,i] = [x0[0]-0.125*j*step_max, x0[1]+0.25*j*step_max]
                #x_guess(j,:,i) = [x0(1) + 0.125*(j-j**2/(num_path*4))*step_max, x0(2) + 0.125*j*step_max]
                
            elif i == 5: #to the right, slightly down
                x_guess[j,:,i] = [x0[0]+0.125*j*step_max, x0[1]+0.0625*j*step_max]
                
            elif i == 6: #to the top, slightly left
                x_guess[j,:,i] = [x0[0]+0.0625*j*step_max, x0[1]+0.125*j*step_max]
                
            else:
                break #outside of range, need to initialize more guesses (do randomly maybe?)
    return x_guess