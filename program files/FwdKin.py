import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import randint

# make numpy raise errors instead of warn so can be caught by try except blocks
np.seterr(all="raise")

# plot axis origins onto point with rotation
def plotOrigin(tx, ty, tz, rx, ry, rz, length):
    # transform origin
    rotall = np.matmul(np.matmul(Rz(rz), Ry(ry)), Rx(rx))
    xplots = np.matmul(rotall, ((0, length), (0, 0), (0, 0)))
    yplots = np.matmul(rotall, ((0, 0), (0, length), (0, 0)))
    zplots = np.matmul(rotall, ((0, 0), (0, 0), (0, length)))
    xplots[0] += tx
    xplots[1] += ty
    xplots[2] += tz
    yplots[0] += tx
    yplots[1] += ty
    yplots[2] += tz
    zplots[0] += tx
    zplots[1] += ty
    zplots[2] += tz

    # plot origin
    ax.plot(*xplots, c="#ff0000")
    ax.plot(*yplots, c="#00ff00")
    ax.plot(*zplots, c="#0000ff")


# find the position of point given a vector
def findPoint(start, length, theta1, theta2):
    end = [0, 0, 0]
    end[0] = start[0] + length * np.cos(theta1) * np.cos(theta2)
    end[1] = start[1] + length * np.cos(theta1) * np.sin(theta2)
    end[2] = start[2] + length * np.sin(theta1)
    return end


# define rotation matrices
def Rx(a):
    return ((1, 0, 0), (0, np.cos(a), -np.sin(a)), (0, np.sin(a), np.cos(a)))


def Ry(a):
    return ((np.cos(a), 0, np.sin(a)), (0, 1, 0), (-np.sin(a), 0, np.cos(a)))


def Rz(a):
    return ((np.cos(a), -np.sin(a), 0), (np.sin(a), np.cos(a), 0), (0, 0, 1))


# function to find angle between two vectors
def getAngle(vect1, vect2):
    try:
        temp1 = np.linalg.norm(vect1)
        temp2 = np.linalg.norm(vect2)
        unitvect1 = vect1 / temp1
        unitvect2 = vect2 / temp2
        theta = np.arccos(np.clip(np.dot(unitvect1, unitvect2), -1.0, 1.0))
        return theta
    except Exception:
        return np.pi / 2 if temp2 > 0 else -np.pi / 2


# linear interpolation
def lerp(a, b, t):
    return (a * (1 - t)) + (b * t)


def fwdKin(lengths, angles):
    # trace the path to find plot points
    xplots = [0, 0]
    yplots = [0, 0]
    zplots = [0, lengths[0]]
    mainlength1 = lengths[1]
    mainlength2 = lengths[2] + lengths[3]
    mainlength3 = lengths[4] + lengths[5]
    theta1 = (np.pi / 2) - angles[1]
    theta2 = angles[0]
    point = findPoint([xplots[1], yplots[1], zplots[1]], mainlength1, theta1, theta2)
    xplots.append(point[0])
    yplots.append(point[1])
    zplots.append(point[2])
    theta1 = (np.pi / 2) - angles[1] - angles[2]
    theat2 = angles[0]
    point = findPoint([xplots[2], yplots[2], zplots[2]], mainlength2, theta1, theta2)
    savedpoint = point  # save the point for later derivation
    xplots.append(lerp(xplots[2], point[0], lengths[2] / mainlength2))
    yplots.append(lerp(yplots[2], point[1], lengths[2] / mainlength2))
    zplots.append(lerp(zplots[2], point[2], lengths[2] / mainlength2))
    xplots.append(point[0])
    yplots.append(point[1])
    zplots.append(point[2])
    transformed = np.array([mainlength3, 0, 0])
    transformed = np.matmul(Ry(angles[4]), transformed)
    transformed = np.matmul(Rx(angles[3]), transformed)
    transformed = np.matmul(Ry(np.pi + theta1), transformed)
    transformed = np.matmul(Rz(np.pi + theta2), transformed)
    point = transformed + point
    xplots.append(lerp(xplots[4], point[0], lengths[4] / mainlength3))
    yplots.append(lerp(yplots[4], point[1], lengths[4] / mainlength3))
    zplots.append(lerp(zplots[4], point[2], lengths[4] / mainlength3))
    xplots.append(point[0])
    yplots.append(point[1])
    zplots.append(point[2])

    # convert the data type
    xplots = np.array(xplots)
    yplots = np.array(yplots)
    zplots = np.array(zplots)

    # we now know the end position
    tx = xplots[-1]
    ty = yplots[-1]
    tz = zplots[-1]

    # derive theta1 and theta2 of wrist vector
    # define root vectors
    wrstvect = point - savedpoint

    txdelta = wrstvect[0]
    tydelta = wrstvect[1]
    tzdelta = wrstvect[2]

    elevvect = np.array([txdelta, tydelta, 0])
    azimvect = np.array([txdelta, 0, tzdelta])

    elevangle = getAngle(elevvect, wrstvect)
    azimangle = getAngle(azimvect, wrstvect)

    if tzdelta < 0:
        elevangle *= -1

    if tydelta < 0:
        azimangle *= -1

    rx = angles[5]
    ry = -elevangle
    rz = azimangle

    return ((tx, ty, tz, rx, ry, rz), xplots, yplots, zplots)


def main():
    global ax

    # create and configure plot object
    fig = plt.figure(figsize=(8, 8))
    fig.subplots_adjust(left=0.01, bottom=0.01, right=0.96, top=0.96)
    ax = fig.add_subplot(111, projection="3d")
    # ax.set(xlim=(-260,260), ylim=(-260,260), zlim=(0,400))
    ax.set(xlim=(0, 260), ylim=(0, 260), zlim=(0, 200))
    ax.w_xaxis.line.set_color("#ff0000")
    ax.w_yaxis.line.set_color("#00ff00")
    ax.w_zaxis.line.set_color("#0000ff")
    ax.set_xlabel("x-axis")
    ax.set_ylabel("y-axis")
    ax.set_zlabel("z-axis")
    # ax.view_init(elev=30, azim=-135)
    ax.view_init(elev=30, azim=135)

    lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

    angles = np.array([60, 30, 90, 0, 0, 0])

    raw = fwdKin(lengths, np.radians(angles))
    tx = raw[0][0]
    ty = raw[0][1]
    tz = raw[0][2]
    rx = raw[0][3]
    ry = raw[0][4]
    rz = raw[0][5]
    xplots = raw[1]
    yplots = raw[2]
    zplots = raw[3]

    # plot data
    ax.plot(xplots, yplots, zplots, c="#000000")
    ax.scatter(xplots, yplots, zplots, c="#00ffff")
    plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
    plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)

    # output calculated position
    print(f"Tx = {round(tx,2)} mm")
    print(f"Ty = {round(ty,2)} mm")
    print(f"Tz = {round(tz,2)} mm")
    print(f"Rx = {round(np.degrees(rx),2)} deg")
    print(f"Ry = {round(np.degrees(ry),2)} deg")
    print(f"Rz = {round(np.degrees(rz),2)} deg")

    # show plotted data
    plt.show()


if __name__ == "__main__":
    main()
