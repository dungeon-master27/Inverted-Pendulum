import numpy as np
import cv2
from free_fall_pendulum import InvertedPendulum
from scipy.integrate import solve_ivp

def func2( t, y ):
    g = 9.8 
    L = 1.5 
    m = 1.0
    M = 5.0 
    x_ddot = L * y[3]*y[3] * np.cos( y[2] )  -  g * np.cos(y[2]) *  np.sin(y[2])
    x_ddot = m / ( m* np.sin(y[2])* np.sin(y[2]) - M -m ) * x_ddot
    theta_ddot = -g/L * np.cos( y[2] ) - 1./L * np.sin( y[2] ) * x_ddot
    damping_theta =  - 0.5*y[3]
    damping_x =  - 1.0*y[1]
    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]
    pass

if __name__=="__main__":

    sol = solve_ivp(func2, [0, 20], [ -1.0, 0., np.pi/2 + 0.1, 0. ],   t_eval=np.linspace( 0, 20, 300)  )
    syst = InvertedPendulum()

    for i, t in enumerate(sol.t):
        rendered = syst.step( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        cv2.imshow( 'im', rendered )
        if cv2.waitKey(30) == ord('q'):
            break
