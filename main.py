import numpy as np
import scipy
from scipy import optimize

from CalculateEnergyUse import calc_eff
from functions import *

class GLOBALS():
    xf = None # Final Position
    x0 = None # Current Starting Point - Path_bez
    step_max = None # Max Step Distance
    step_min = None # Min Step Distance
    t = None #parameterization variable
    n_obs = None # number of obstacles
    obs = None # positions of obstacles
    obs_rad = None # radius of obstacles
    turn_r = None # minimum turn radius
    Pmid = None # needed to match derivatives
    num_path = None # number of segments optimized
    x_new = None
    Dynamic_Obstacles = None
    x_next = None # used in multi_start function
    uav_ws = None # UAV wing span
    start = None
    initial = None # to calculate d_l_min
    initial = 1
    uav_finite_size = None
    rho, f, W, span, eo = None, None, None, None, None
    # summer_c, cool_c, copper_c, parula_c, winter_c, blue_red, blue_magenta_red, green_purple, blue_gray_red, shortened_viridis_c, shortened_inferno_c, shortened_parula_c
    obj_grad, cons_grad, ag, acg = None, None, None, None
    max_speed, min_speed, D_eta_opt = None, None, None
    l_l_last = None

############### ----- Algorithm Options ----- ###############
GLOBALS.Dynamic_Obstacles = 0 # Set to 1 to use dynamic obstacles, 0 for only static obstacles

GLOBALS.num_path = 3 # Receding Horizon Approach (can be any number, 3 is fairly standard. Represents how many path segments ahead the algorithm will look when planning a path)
ms_i = 3 # Number of guesses for multi start (up to 8, 3 is good for a smart algorithm)
GLOBALS.uav_finite_size = 1 # Whether or not to include the UAV size (1 to include, 0 to ignore)
check_viability = 1 # Check if the path is viable. When set to 1 the code will exit if the path isn't viable

### Objective Function Options
optimize_energy_use = 1 # 1 to select, 0 to ignore
optimize_time = 0 # 1 to select, 0 to ignore
# NOTE: If both optimize_energy_use and optimize_time are set to 1 then the algorithm will optimize path length
max_func_evals = 10000
max_iter = 50000

### Plot Options
totl = 1 # Turn off tick labels
square_axis = 1 # Ensure squared axis
radar = 0 # Plots UAV limit of sight
show_sp = 0 # Plots P1 of Bezier Curve

Show_Steps = 0 # If Dynamic_Obstacles is set to 1 then this must also be set to 1
linewidth = 4
traversedwidth = 2
dashedwidth = 2

fwidth = 2
show_end = 0
compare_num_path = 0
save_path = 0
sds = 0
cx = 50

#plot color options
speed_color = 1         #use if you want color to represent speed
d_speed_color = 0       #use if you want color to be discretized over path length
cb = 1                  #color brightness

summer_c = 0             # http://www.mathworks.com/help/matlab/ref/colormap.html#buq1hym
cool_c = 0
copper_c = 0
parula_c = 0
winter_c = 0
blue_red = 0
blue_magenta_red = 0
green_purple = 0
blue_gray_red = 0
shortened_parula_c = 1
shortened_viridis_c = 0
shortened_inferno_c = 0
color_bar = 0

#----------------------------------------#

create_video = 1          #saves the solutions of the multistart approach at each iteration

# Gradient Calculation Options
GLOBALS.obj_grad = 1           #if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_gradients = 1
GLOBALS.ag = analytic_gradients

GLOBALS.cons_grad = 1          #if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_constraint_gradients = 1
GLOBALS.acg = analytic_constraint_gradients

#----------------plane geometry/info----------------#
#UAV parameter values
GLOBALS.rho = 1.225 #air density
GLOBALS.f = .2   #equivalent parasite area
GLOBALS.W = 10 #weight of aircraft
GLOBALS.span = .20   #span
GLOBALS.eo = 0.9 #Oswald's efficiency factor

if optimize_energy_use == 1:
    #Defined in paper (2nd column, page 2)
    A = GLOBALS.rho*GLOBALS.f/(2*GLOBALS.W)
    B = 2*GLOBALS.W/(GLOBALS.rho*GLOBALS.span**2*np.pi*GLOBALS.eo)
    
    #find minimum d_l, and minimum efficiency
    if GLOBALS.initial == 1:
        V_possible = np.arange(0.1,25.01,0.01)
        D_eta = [0] * len(V_possible)
        for i in range(0,len(V_possible)):
            V_possible[i] = round(V_possible[i], 2)
            D_L = A*V_possible[i]**2 + B/V_possible[i]**2 # we want to maximize l_d, or minimize d_l
            
            eta_pos = calc_eff(V_possible[i])
            
            #calculate D_L/eta
            D_eta[i] = D_L/eta_pos
        
        #find optimal D_eta
        GLOBALS.D_eta_opt = min(D_eta)   
    
# --------------------------------- #

l = 0

# Parameterization vector t
GLOBALS.t = np.linspace(0,1,11)
delta_t = GLOBALS.t[1] - GLOBALS.t[0] # NOTE if you have a pylinter error on this line, it is false. It works just fine so just ignore it. :)

GLOBALS.turn_r = 40 # Turn radius, m

# Maximum / stall speed, m/s
GLOBALS.max_speed = 23
GLOBALS.min_speed = 16

GLOBALS.l_l_last = GLOBALS.min_speed / len(GLOBALS.t)

# Translate UAV information to fit with algorithm
GLOBALS.step_max = GLOBALS.max_speed
GLOBALS.step_min = GLOBALS.min_speed

# Wing Span of UAV
if GLOBALS.uav_finite_size:
    GLOBALS.uav_ws = 1.91 # UAV wing span
else:
    GLOBALS.uav_ws = 0.001

# Starting / Ending position of the plane
x_sp = [0,0]
GLOBALS.x0 = x_sp
GLOBALS.xf = [100, 100]
Bez_points = []
lr = 15 # Landing zone radius (should be <= 15??)

#--------------------------------------------------#

#-------static obstacle information---------#
import time
start = time.time()
from metis.generator import MissionGenerator
mission = MissionGenerator().get_mission()
print("time to generate mission:", time.time() - start)
GLOBALS.n_obs = len(mission.obstacles) # Number of static obstacles
GLOBALS.obs = [] # Obstacle locations
GLOBALS.obs_rad = [] # Obstacle radii
for obs in mission.obstacles:
    n, e, _, r = obs.to_array()
    GLOBALS.obs.append([n, e])
    GLOBALS.obs_rad.append(r)
GLOBALS.obs = np.array(GLOBALS.obs)
GLOBALS.obs_rad = np.array(GLOBALS.obs_rad)
#--------------------------------------------------#

# Calculate density
start = time.time()
obs_density = calc_obs_den(GLOBALS.n_obs, GLOBALS.obs, GLOBALS.obs_rad, GLOBALS.uav_ws)
print("time to calculate obstacle density:", time.time() - start)

#-------dynamic obstacle information---------#
if GLOBALS.Dynamic_Obstacles:
    pleaseDeleteMeAndIncorporateThisFunction = 1
    # FIXME need to incorporate other telemetry signals here

#----------------- optimizer ---------- fmincon -----------------------#

# NOTE most of the code in this block will be different from the matlab code
GLOBALS.Pmid = [-(GLOBALS.min_speed/2), -(GLOBALS.min_speed/2)]

Path_bez = []
path_start = []

GLOBALS.start = 0 # Used in multi_start, indicates that the program is in the first iteration
xi = multi_start(ms_i)
GLOBALS.start = 1
