import numpy as np


def generate_velocity_profile(path, initial_velocity):
    l = len(path)
    v = deltav = theta = [0.0]*l
    deltav[0] = 0.0
    initialVelocity = initial_velocity
    v[0] = initialVelocity
    w, h = 3, l
    waypoints = [[0.0 for xx in range(w)] for yy in range(h-1)]
    arr = np.array([0, 0, 0, 0])
    x = y = z = [0.0]*l
    t, a, b, c = 0, 0.0, 0.0, 0.0
    x, y = path[:, 0], path[:, 1]
    waypoints[0][0] = x[0]
    waypoints[0][1] = y[0]
    waypoints[0][2] = v[0]
    """
    for counted in range(h-1):
        if counted > 0:
            x[counted-1] = float(x[counted])
            y[counted-1] = float(y[counted])
    """
    for xx in range(1, l-1):

        m = 730.0
        mu = 0.6
        rollingFriction = 65.0
        N = m*9.81*np.cos(theta[xx]) + 0.96552*np.square(v[xx])
        d = 4.0
        u = v[xx-1]
        a = np.sqrt(np.square(x[xx]-x[xx-1]) +
                    np.square(y[xx]-y[xx-1])+np.square(z[xx]-z[xx-1]))
        b = np.sqrt(np.square(x[xx]-x[xx+1]) +
                    np.square(y[xx]-y[xx+1])+np.square(z[xx]-z[xx+1]))
        c = np.sqrt(np.square(
            x[xx+1]-x[xx-1])+np.square(y[xx+1]-y[xx-1])+np.square(z[xx+1]-z[xx-1]))

        if (a+b+c)*(a+b-c)*(a+c-b)*(b+c-a) == 0:
            R = 1000000000000000000.0
        else:
            R = a*b*c/(np.sqrt((a+b+c)*(a+b-c)*(a+c-b)*(b+c-a)))

        # sqrt(np.square(m*np.square(v1)/R-m*9.81*sin(theta[x])) + np.square(deltav[x]*(v[x]+v[x-1])/(2*d) + rollingFriction + 0.445*np.square(v[x-1]))) = mu*N #gives delta
        arr = np.roots([np.square(m/R) + np.square(m)/(np.square(d)*4) - np.square(0.96552), 0, -2*np.square(m)*9.81*np.sin(theta[xx])/R + m*(rollingFriction+0.445*np.square(u)-m*np.square(u)/(2*d)) /
                        d - 2*0.96552*mu*m*9.81*np.cos(theta[xx]), 0, np.square(m*9.81*np.sin(theta[xx])) + np.square(rollingFriction+0.445*np.square(u)-m*np.square(u)/(2*d)) - np.square(mu*m*9.81*np.cos(theta[xx]))])
        arr = np.sort(arr)
        delta = arr[3]-v[xx-1]

        if delta >= (np.sqrt(np.square(v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]) - 1550.0)/m)-v[xx-1]):
            deltav[xx] = (np.sqrt(np.square(
                v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]) - 1550.0)/m)-v[xx-1])
        else:
            if delta < (np.sqrt(np.square(v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]))/m)-v[xx-1]):
                b = 2
                for changes in range(1, b):
                    u = np.sqrt(np.square(np.sqrt(
                        R*(mu*N/m+9.81*np.sin(theta[xx]))))-2*d*(rollingFriction + 0.445*np.square(v[xx-1]))/m)
                    for a in range(1, changes+1):
                        deltav[xx-changes+a-1] = max(deltav[xx-changes+a-1]-(v[xx-1]-u)/(changes), (np.sqrt(
                            np.square(v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]))/m)-v[xx-1]))
                    # find delta again
                    if delta < (np.sqrt(np.square(v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]))/m)-v[xx-1]):
                        if xx > (b-1):
                            b += 1
                    else:
                        if delta >= (np.sqrt(np.square(v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]) - 1550.0)/m)-v[xx-1]):
                            deltav[xx] = (np.sqrt(np.square(
                                v[xx-1])-2*d*(rollingFriction + 0.445*np.square(v[xx-1]) - 1550.0)/m)-v[xx-1])
                        else:
                            deltav[xx] = delta
            else:
                deltav[xx] = delta
        # deltav[xx]= min(max((sqrt(np.square(vxminus1)+2*d*(rolling friction + 0.441*np.square(v))/m)-v), delta),(sqrt(np.square(vxminus1)+2*d*(rolling friction + 0.441*np.square(v) + ****)/m)-v))
        v[xx] = v[xx-1] + deltav[xx]

        waypoints[xx][0] = x[xx]
        waypoints[xx][1] = y[xx]
        waypoints[xx][2] = v[xx]
    #waypoints = waypoints[0:-1]
    return waypoints


x = np.arange(1, 100)
paths = np.concatenate((x.reshape(-1, 1), x.reshape(-1, 1)), axis=1)
inivel = 42
velopro = generate_velocity_profile(paths, inivel)
print(velopro)
