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