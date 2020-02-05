from roblib import *

def dynamique(X, u):
    X = X.flatten()
    u = u.flatten()
    u1, u2 = u
    x, y, theta, v = X
    xdot = v*cos(theta)
    ydot = v*sin(theta)
    thetadot = u1 - u2
    vdot = u1 + u2 - abs(v)*v
    return array([[xdot, ydot, thetadot, vdot]]).T


def observateur(X,Xb, Gx, dt):
    Xf = X.flatten()
    x, y, theta, v = Xf
    u1 = v*cos(theta)
    u2 = v*sin(theta)
    u = dt*array([[u1, u2]]).T
    A = eye(2)
    C = eye(2) 
    Galpha = zeros((2,2))
    Gbeta = 100*eye(2)
     
    Y = array([[x, y]]).T
    Xb, Gx = kalman(Xb, Gx, u, Y, Galpha, Gbeta, A, C)
    return Xb, Gx

if __name__ == "__main__" :
    X = array([[5, 5, 0.1, 0.]]).T
    u_c = array([[0.8, 0.9]]).T
    dt = 1
    ax = init_figure(-40, 40, -40, 40)
    
    Xb = array([[2, 2]]).T
    Gx = 1000*eye(2)
    
    for t in arange(0, 300, dt):
        clear(ax)
        Xb, Gx = observateur(X,Xb, Gx, dt)
        draw_ellipse(Xb,Gx,0.9,ax,[0,1,0])
        X = X + dt * dynamique(X, u_c)
        scatter(Xb[0], Xb[1])
        draw_boat(X, r=0.5, col="teal")
        