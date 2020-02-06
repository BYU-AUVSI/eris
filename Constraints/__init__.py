import sys
sys.path.append('..')

import numpy as np

from main import GLOBALS

from . import calc_cons

def cons(xi):
    """
    Calculates constraints.
    
    Parameters
    ----------
    xi :

    Returns
    -------
    c : 
    ceq : 
    gc : 
    gceq : 
    """

    num_path, cons_grad, acg = GLOBALS.num_path, GLOBALS.cons_grad, GLOBALS.acg

    c = []
    ceq = []
    gc = []
    gceq = []

    if acg == 0:
        (c, ceq) = calc_cons(xi)

    if (cons_grad == 1) and (acg == 0):
        h = 10**(-20)
        gc = np.zeros((num_path*4,length(c)))
        gceq = np.zeros((num_path*4,length(ceq)))
        
        for i in range(num_path*2):# = 1 : num_path*2
            for j in range(2): # = 1 : 2
                
                xi[i,j] = xi[i,j] + 1j*h
                
                #calculate c gradients using complex step
                ci, _ = calc_cons(xi)
                
                gc[2*num_path*(j)+i, :] = np.imag(ci)/h
                
                #calculate ceq gradients using complex step
                _, ceqi = calc_cons(xi)
                
                gceq[2*num_path*(j)+i, :] = np.imag(ceqi)/h
                
                xi[i,j] = xi[i,j] - 1j*h
        
    elif cons_grad == 1 and acg == 1:
        [c, ceq, gc, gceq] = calc_cons_acg(xi)
        
    #     #curvature constraints
    #     curvature_c = curvature_cons(xi)
    #     
    #     c = [c curvature_c]

    #gradients of curvature constraints calculated using finite difference
    # dh = 10.0**(-6.0)
    # 
    # curvature_gc = zeros(4*num_path,27)
    # 
    # for i = 1 : num_path*2
    #     
    #     for j = 1 : 2
    #         
    #         xi(i,j) = xi(i,j) + dh
    #         
    #         c_x_plus_h = curvature_cons(xi)
    #         
    #         xi(i,j) = xi(i,j) - dh
    #         
    #         c_x = curvature_cons(xi)
    #         
    #         curvature_gc(2*num_path*(j-1)+i,:) = (c_x_plus_h - c_x)/dh
    #         
    #         
    #     end
    #     
    # end
    # 
    # gc = [gc curvature_gc]

    c = np.real(c)
    return c, ceq, gc, gceq