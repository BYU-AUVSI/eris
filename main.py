import time

import numpy as np
import scipy
from scipy import optimize

from CalculateEnergyUse import calc_eff
from functions import *

from glo import GLOBALS

############### ----- Algorithm Options ----- ###############
# use genetic algorithm
use_ga = False

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
GLOBALS.max_speed = 15 #23
GLOBALS.min_speed = 10 #16

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
x_sp = [-500,-500]
GLOBALS.x0 = x_sp
GLOBALS.xf = [500, 500]
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
# start = time.time()
# obs_density = calc_obs_den(GLOBALS.n_obs, GLOBALS.obs, GLOBALS.obs_rad, GLOBALS.uav_ws)
# print("time to calculate obstacle density:", time.time() - start)

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

# import matplotlib.pyplot as plt
# for i in range(ms_i):
#     plt.plot(xi[:,0,i], xi[:,1,i])
# plt.show()

#x_new is not close to final position
x_new = np.zeros((2*GLOBALS.num_path,2))
x_next = x_new

# note: each iteration of while loop represents some time step, in which
# UAV travels on path and dynamic obstacles move

# #fmincon options
# if GLOBALS.obj_grad == 1 and GLOBALS.cons_grad == 1:
#     options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter, \
#         'GradObj','on','GradCon','on','DerivativeCheck','off')
# elif GLOBALS.obj_grad == 0 and GLOBALS.cons_grad == 1:
#     options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter, \
#         'GradObj','off','GradCon','on','DerivativeCheck','off')
# elif GLOBALS.obj_grad == 1 and GLOBALS.cons_grad == 0:
#     options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter, \
#         'GradObj','on','GradCon','off','DerivativeCheck','off')
# else:
#     options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter, \
#         'GradObj','off','GradCon','off')

tic = time.time() # begin optimization time



import sys
sys.exit()

while ( ( (x_next[2*GLOBALS.num_path - 1,0]-GLOBALS.xf[0])**2. + (x_next[2*GLOBALS.num_path - 1,1]-GLOBALS.xf[1])**2. ) ** 0.5 > lr ):
    #record number of paths
    l = l + 1
    break;

#     for i in range(ms_i): #multistart approach to find best solution
        
#         #choose objective function
#         if optimize_energy_use == 1:
            
#             if use_ga:
#                 NotImplementedError('Not ready yet for ga')
#             else:
#                 [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_e, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options)
            
#         elif optimize_time == 1:
            
#             if use_ga:
#                 NotImplementedError('Not ready yet for ga')
#             else
#                 [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_t, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options)
            
#         else
            
#             if use_ga == 1
#                 NotImplementedError('Not ready yet for ga')
#             else
#                 [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_d, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options)
        
#         #         #check curvature
#         #         c = check_curvature_new(i)
#         #
#         #         #if constraints are violated, make infeasible
#         #         if any(c > 0)
#         #             e(i,l) = -2
#         #         end
#     end
    
#     for i = 1 : ms_i #calculate how good solutions are
        
#         # For make_video
#         if create_video == 1
            
#             if i == 1
#                 solution1 = [solution1 x_new(:,:,i)]
#             elif i == 2
#                 solution2 = [solution2 x_new(:,:,i)]
#             elif i == 3
#                 solution3 = [solution3 x_new(:,:,i)]
#             elif i == 4
#                 solution4 = [solution4 x_new(:,:,i)]
#             elif i == 5
#                 solution5 = [solution5 x_new(:,:,i)]
#             end
#         end
        
#         if optimize_energy_use == 1
#             d_check(i) = opt_e(x_new(:,:,i))
            
#         elif optimize_time == 1
#             d_check(i) = opt_t(x_new(:,:,i))
            
#         else
#             d_check(i) = opt_d(x_new(:,:,i))
            
#         end
        
#         #'remove' solutions that converged to an infeasible point
        
#         if e(i,l) == -2
            
#             d_check(i) = d_check(i)*10
            
#         end
        
        
#     end
    
#     #Check for viable paths
#     check = (e == -2)
#     if all(check(:,l)) == 1 and check_viability == 1
#         #error('Unable to find viable path.')
#     end
    
#     for i = 1 : ms_i #choose best solution, use for next part
        
#         if d_check(i) == min(d_check)
            
#             x_next = x_new(:,:,i)
            
#         end
#     end
    
#     #
#     initial = 0
    
#     # makes the path of the UAV for this section
#     for i = 1 : length(t)
        
#         path_part(i,:) = (1-t(i)) **2*x0(1,:) + 2*(1-t(i))*t(i)*x_next(1,:)+t(i) **2*x_next(2,:)
        
#         #         if i > 1
#         #         norm(path_part(i,:)-path_part(i-1,:))
#         #         end
        
#     end
    
    
    
    
#     #make the planned path of the UAV
#     if num_path > 1
#         for j = 1 : (num_path-1)
#             for i = 1 : length(t)
#                 path_planned(i+(j-1)*length(t),:) = (1-t(i)) **2*x_next(2*j,:) + 2*(1-t(i))*t(i)*x_next(2*j+1,:)+t(i) **2*x_next(2*j+2,:)
#             end
#         end
#     end
    
#     if Show_Steps == 1
        
#         plot_int_steps(l, square_axes, color_bar, totl, x_sp, cx, speed_color, path_part, path_planned, Path_bez, d_speed_color, cb \
#             ,linewidth, traversedwidth, dashedwidth, radar, show_sp, show_end, sds, lr)
#     end
    
#     #----------------------------------------------------------#
    
#     #record where start of each path is
#     path_start = [path_start path_part(1,:)]
    
#     #continues the path which will be plotted
#     Path_bez = [Path_bez path_part]
    
#     l_l_last = norm(Path_bez(length(Path_bez),:)-Path_bez(length(Path_bez)-1,:))
    
#     #set new starting point
#     x0 = x_next(2,:)
    
#     #set Pmid
#     Pmid = x_next(1,:)
    
#     #choose new guess for next iteration
#     xi = multi_start(ms_i)
    
#     #print current location
#     x_next(2,:)
    
#     Bez_points = [Bez_points x_next(1:2,:)]
    
# end #while

# toc # end optimization time