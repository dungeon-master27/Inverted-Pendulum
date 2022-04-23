import numpy as np
import cv2
from free_fall_pendulum  v import InvertedPendulum
from scipy.integrate import solve_ivp

def u( t ):
    t1 = 3.
    t2 = 5.

    if( t > t1 and t < t2 ):
        return 10.
    else:
        return 0.

def func3( t, y ):
    g = 9.8 
    L = 1.5
    m = 1.0 
    M = 5.0 

    x_ddot = u(t) - m*L*y[3]*y[3] * np.cos( y[2] ) + m*g*np.cos(y[2]) *  np.sin(y[2])
    x_ddot = x_ddot / ( M+m-m* np.sin(y[2])* np.sin(y[2]) )
    theta_ddot = -g/L * np.cos( y[2] ) - 1./L * np.sin( y[2] ) * x_ddot
    damping_theta =  - 0.5*y[3]
    damping_x =  - 1.0*y[1]
    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]



if __name__=="__main__":

    sol = solve_ivp(func3, [0, 20], [ -1.0, 0., np.pi/2 - 0.1, 0. ],   t_eval=np.linspace( 0, 20, 300)  )
    syst = InvertedPendulum()
    for i, t in enumerate(sol.t):
        rendered = syst.step( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        cv2.imshow( 'im', rendered )
        cv2.moveWindow( 'im', 100, 100 )

        if cv2.waitKey(30) == ord('q'):
            break
