from roblib import *

dt = 0.01

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


def control(x, w, dw):
    xf = x.flatten()
    w = w.flatten()
    dw = dw.flatten()
    

    px, py, theta, v = xf
    ct = cos(theta)
    st = sin(theta)
    A = array([[ct - v*st, ct + v*st], [st + v*ct, st -v*ct]])
    B = np.array([[-abs(v)*v*ct],[-abs(v)*v*st]])

    v1 = 1*(w[0] - px) + 2*(dw[0]  - v*ct)
    v2 = 1*(w[1] - py) + 2*(dw[1]  - v*st)


    z = array([[v1], [v2]])
    u =  inv(A)@(z - B)

    return u


def set_point(t):
   
    omega = 0.5
    w = array([[R*cos(omega*t)], 
               [R*sin(omega*t)]])
    dw = array([[-omega*R*sin(omega*t)], 
                [R*omega*cos(omega*t)]]) 
    return w, dw 


X = array([[0.1, 0.1, 0.1, 0.1]]).T

theta = linspace(0,  2*pi, 100)

R = 5
cx = R*cos(theta)
cy = R*sin(theta)

ax = init_figure(-10, 10, -10, 10)
u = array([[0, 0]]).T

for t in arange(0, 10, dt):
    clear(ax)
    w, dw = set_point(t)
   
    u = control(X, w, dw)
    xdot = dynamique(X, u)
    X = X + xdot*t
    plot(cx, cy)
    scatter(w[0], w[1])
    draw_tank(X)
    