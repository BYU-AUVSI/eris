import numpy as np
import matplotlib.pyplot as plt

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


# Plot Drag
# http://cafefoundation.org/v2/pdf_tech/MPG.engines/AIAA.1980.1847.B.H.Carson.pdf

def plot_drag():
    # Check to see how cruise efficiency is affected by velocity

    # Parameter values
    rho = 1.225 #air density
    f = 0.2   #equivalent parasite area
    W = 10 #weight of aircraft
    b = 0.20   #span
    e = 0.9 #Oswald's efficiency factor

    #velocity
    v = np.arange(0.01, 30, 0.1)

    #Defined in paper (2nd column, page 2)
    A = rho*f/(2*W)
    B = 2*W/(rho*b**2*np.pi*e)

    #vehicle specific power (2nd column, page 4)
    #epsilon = 1
    # https://en.wikipedia.org/wiki/Vehicle-specific_power
    # http://cires1.colorado.edu/jimenez/Papers/Jimenez_VSP_9thCRC_99_final.pdf

    #defined in paper (1st column, page 5)
    # for i = 1 : length(v)
    #

    #cruise efficiency
    #     k(i) = epsilon(i)/v(i)
    #
    # end
    # for i = 1 : length(v)
    #
    #     C(i) = 0.57*k(i)*(A**3*B)**(-1/4)
    #
    # end

    #calculate l_d
    d_l = np.zeros(len(v))
    l_d = np.zeros(len(v))

    for i in range(len(v)):
        d_l[i] = A*v[i]**2 + B/v[i]**2
        l_d[i] = d_l[i]**(-1)

    plt.figure()
        # set(gca,'FontSize',20)
        # set(gca,'FontSize',20)

    plt.plot(v,d_l)#,'LineWidth',2)
    plt.xlabel('UAV Velocity (m/s)')#,'fontsize',18)
    plt.ylabel('Drag (N)')#,'fontsize',18)
    #title('Drag vs. UAV Speed')
    plt.ylim(0, 40)

    #plot propulsive efficiency
    V = np.arange(0.01, 30, 0.1)

    a = -0.0024
    b = 0.084
    c = -0.9
    d = 3.6

    eta = np.zeros(len(V))

    for i in range(len(V)):
        
        if V[i] < 10:
            eta[i] = V[i]*0.06
        elif V[i] < 20:
            eta[i] = a*V[i]**3 + b*V[i]**2 + c*V[i] + d
        else:
            eta[i] = 0.00001

    plt.figure()

        # set(gca,'FontSize',20)
        # set(gca,'FontSize',20)

    plt.plot(V,eta)#,'LineWidth',2)
    #title('Efficiency vs. UAV Speed')
    plt.xlim(0, 21)
    plt.ylim(0, 1)
    plt.xlabel('UAV Velocity (m/s)')#,'fontsize',18)
    plt.ylabel('Overall Propulsive Efficiency')#,'fontsize',18)

    #-----------------------------------------#

    opt = np.zeros(len(V))
    for i in range(len(V)):
        opt[i] = d_l[i]/eta[i]


    plt.figure()

        # set(gca,'FontSize',20)
        # set(gca,'FontSize',20)

    plt.plot(V,opt)
    plt.ylim(0, 20)
    plt.xlim(5, 20)
    
    plt.show()
