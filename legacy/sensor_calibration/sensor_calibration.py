
import numpy as np
import cv2
import matplotlib.pyplot as plt

import GPy


def get_table(frame, rad = 51.0/2):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    lsd = cv2.ximgproc.createFastLineDetector()
    # Detect lines in the image
    lines = lsd.detect(img)  # Position 0 of the returned tuple are the detected lines

    print(lines.shape)

    # Draw detected lines in the image
    drawn_img = lsd.drawSegments(img, lines)

    lines = np.squeeze(lines)
    print(lines.shape)
    #lines = lines.reshape(32,4)
    line_assort = np.zeros((16,5))
    idx = np.argsort(lines[:,0], axis = 0)
    lines = lines[idx,:]

    # radius of tactile sensor
    #rad = 51.0/2
    # line_assort[:,0]: collects radius of indication in image frame
    # line_assort[:,1]: collects the length of 2mm bar in image frame
    # line_assort[:,2]: defines the corresponding radius of indication in real frame

    center_x = (lines[0,0] + lines[0,2] + lines[0+1,0]+lines[0+1,2])/4
    center_y = (lines[0,1] + lines[0,3] + lines[0+1,1]+lines[0+1,3])/4
    print("center_x : ", center_x, "  center_y: ", center_y)

    for i in range(16):
      idx = 2*i
      line_assort[i, 0] = (lines[idx,0] + lines[idx,2] + lines[idx+1,0]+lines[idx+1,2])/4 - center_x
      line_assort[i, 1] = np.abs(lines[idx,1] - lines[idx,3])
      theta = i * 5 / 180 * np.pi
      line_assort[i, 2] = rad * np.sin(theta)
      line_assort[i, 3] = theta
      line_assort[i, 4] = i * 5

    savetable = np.zeros((16,2))
    savetable[:, 0] = line_assort[:, 0]
    savetable[:, 1] = line_assort[:, 4]
    data = [savetable, center_x,center_y]
    np.savez('table', lookuptable = savetable, centerx = center_x, centery = center_y)
    data = np.load('table.npz')
    table, center_x, center_y = data['lookuptable'], data['centerx'], data['centery']
    print(savetable)
    return table, center_x, center_y

def get_edge(table):
    data = np.load(table)
    line_assort, center_x, center_y = data['lookuptable'], data['centerx'], data['centery']

    print(line_assort, center_x, center_y)

    # input: r from (u, v), output: Rsin(theta) in real world
    # now (the line displacement from center, and real theta) are using for the gp
    gpsol = draw_gp(line_assort[:, 0], line_assort[:,1], 'xpixel-theta')

    # input: Rsin(theta) in real world, output: r from (u, v)
    gpsol_inv = draw_gp(line_assort[:, 1], line_assort[:,0], 'theta-xpixel(inv)')


    return gpsol, gpsol_inv, center_x, center_y


def draw_gp(X, y, i):


    if i == 'xpixel-theta':
        # using gpy library
        kernel = GPy.kern.RBF(input_dim = 1, variance = 3.0, lengthscale = 40.0)
        X = np.atleast_2d(X).T
        y = np.atleast_2d(y).T
        x = np.atleast_2d(np.linspace(0, np.max(X)*1.2, 100)).T

        model_gp = GPy.models.GPRegression(X,y,kernel)
        model_gp.optimize()
        y_pred, sigma = model_gp.predict(x)

    if i == 'theta-xpixel(inv)':
        # using gpy library
        kernel = GPy.kern.RBF(input_dim = 1, variance = 5.0, lengthscale = 20.0)
        X = np.atleast_2d(X).T
        x = np.atleast_2d(np.linspace(0, np.max(X)*1.2, 100)).T
        y = np.atleast_2d(y).T

        model_gp = GPy.models.GPRegression(X,y,kernel)
        model_gp.optimize()
        y_pred, sigma = model_gp.predict(x)


    plt.figure()
    plt.plot(X, y, 'r.', markersize=10, label='Observations')
    plt.plot(x, y_pred, 'b-', label='Prediction')
    plt.fill(np.concatenate([x, x[::-1]]),
             np.concatenate([y_pred - 1.9600 * sigma,
                             (y_pred + 1.9600 * sigma)[::-1]]),
             alpha=.5, fc='g', ec='None')
    plt.xlabel('$x$')
    plt.ylabel('$f(x)$')
    plt.legend(loc='upper left')

    return model_gp


if __name__ == '__main__':


    fp = "/home/gsznaier/TouchNeRF/data/teddy_bear_touch/nerf_training_data/train/depth/depth_render000.png"
    img = cv2.imread(fp)
    get_table(img)

    #gpsol, gpsol_inv, center_x, center_y = get_edge('table.npz')

    #plt.show()



