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


def control(x, w, dw):
    xf = x.flatten()    

    px, py, theta, v = xf
    ct = np.cos(theta)
    st = np.sin(theta)
    A = np.array([[ct - v*st, ct + v*st], [st + v*ct, st -v*ct]])
    B = np.array([[-abs(v)*v*ct],[-abs(v)*v*st]])
    
    y = np.array([[px], [py]])
    dy = np.array([[v*ct], [v*st]])
    
    v = (w - y) + 2*(dw - dy)
    u =  np.linalg.solve(A, (v - B))
    return u


def set_point(t):
    omega = 0.5
    w = R*array([[cos(omega*t)], [sin(omega*t)]])
    dw = R*array([[-omega*sin(omega*t)], [omega*cos(omega*t)]]) 
    return w, dw 


if __name__ == "__main__" :
    X = array([[0.1, 0.1, 0.1, 0.1]]).T
    u = array([[0, 0]]).T
    
    R = 5
    theta = linspace(0,  2*pi, 100)
    cx = R*cos(theta)
    cy = R*sin(theta)
    
    
    dt = 0.05

    ax = init_figure(-10, 10, -10, 10)

    for t in arange(0, 10, dt):
        clear(ax)
        w, dw = set_point(t)
    
        u = control(X, w, dw)
        X = X + dt * dynamique(X, u)
        plt.plot(cx, cy, color="crimson", alpha=0.4, linewidth=2)
        plt.scatter(w[0], w[1], color="crimson")
        draw_tank(X, r=0.5, col="teal")