import numpy as np

xf # Final Position
x0 # Current Starting Point - Path_bez
step_max # Max Step Distance
step_min # Min Step Distance
t #parameterization variable
n_obs # number of obstacles
obs # positions of obstacles
obs_rad # radius of obstacles
turn_r # minimum turn radius
Pmid # needed to match derivatives
num_path # number of segments optimized
x_new
Dynamic_Obstacles
x_next # used in multi_start function
uav_ws # UAV wing span
start
initial # to calculate d_l_min
initial = 1
uav_finite_size
rho, f, W, span, eo
summer_c, cool_c, copper_c, parula_c, winter_c, blue_red, blue_magenta_red, green_purple, blue_gray_red, shortened_viridis_c, shortened_inferno_c, shortened_parula_c
obj_grad, cons_grad, ag, acg
max_speed, min_speed, D_eta_opt
l_l_last

############### ----- Algorithm Options ----- ###############
Dynamic_Obstacles = 0 # Set to 1 to use dynamic obstacles, 0 for only static obstacles

num_path = 3 # Receding Horizon Approach (can be any number, 3 is fairly standard. Represents how many path segments ahead the algorithm will look when planning a path)
ms_i = 3 # Number of guesses for multi start (up to 8, 3 is good for a smart algorithm)
uav_finite_size = 1 # Whether or not to include the UAV size (1 to include, 0 to ignore)
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
obj_grad = 1           #if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_gradients = 1
ag = analytic_gradients

cons_grad = 1          #if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_constraint_gradients = 1
acg = analytic_constraint_gradients

#----------------plane geometry/info----------------#
#UAV parameter values
rho = 1.225 #air density
f = .2   #equivalent parasite area
W = 10 #weight of aircraft
span = .20   #span
eo = 0.9 #Oswald's efficiency factor

if optimize_energy_use == 1:
    #Defined in paper (2nd column, page 2)
    A = rho*f/(2*W)
    B = 2*W/(rho*span^2*np.pi*eo)
    
    #find minimum d_l, and minimum efficiency
    if initial == 1:
        V_possible = np.arange(0.1,25.01,0.01)
        D_eta = [0] * len(V_possible)
        for i in range(0,len(V_possible)):
            V_possible[i] = round(V_possible[i], 2)
            D_L = A*V_possible[i]**2 + B/V_possible[i]**2 # we want to maximize l_d, or minimize d_l
            
            eta_pos = calc_eff(V_possible[i])
            
            #calculate D_L/eta
            D_eta[i] = D_L/eta_pos
        
        #find optimal D_eta
        D_eta_opt = min(D_eta)   
    
# --------------------------------- #

l = 0

# Parameterization vector t
t = np.linspace(0,1,11)
delta_t = t[1] - t[0]

turn_r = 40 # Turn radius, m

# Maximum / stall speed, m/s
max_speed = 23
min_speed = 16

l_l_last = min_speed / len(t)

# Translate UAV information to fit with algorithm
step_max = max_speed
step_min = min_speed

# Wing Span of UAV
if uav_finite_size:
    uav_ws = 1.91 # UAV wing span
else:
    uav_ws = 0.001







def calc_eff(V):
    a = -0.0024
    b = 0.084
    c = -0.9
    d = 3.6

    if V < 10:
        eta = V*0.06
    elif V < 20:
        eta = a*V**3 + b*V**2 + c*V + d
    else:
        eta = 0.00001
    return eta