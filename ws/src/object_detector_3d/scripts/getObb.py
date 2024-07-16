import numpy as np

import matplotlib.pyplot as plt
import numpy.linalg as LA

# %matplotlib widget # uncomment this if you want to work with this notebook
def draw2DRectangle(x1, y1, x2, y2):
    # diagonal line
    # plt.plot([x1, x2], [y1, y2], linestyle='dashed')
    # four sides of the rectangle
    plt.plot([x1, x2], [y1, y1], color='r') # -->
    plt.plot([x2, x2], [y1, y2], color='g') # | (up)
    plt.plot([x2, x1], [y2, y2], color='b') # <--
    plt.plot([x1, x1], [y2, y1], color='k') # | (down)
    
np.random.seed(22)

OBJECT_WIDTH = 1
OBJECT_HEIGHT = 10
x = np.random.normal(0, OBJECT_WIDTH, (1,100))
y = np.random.normal(0, OBJECT_HEIGHT, (1,100))

theta = 0 # rotation angle
# rotation matrix
rot = lambda theta: np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))],
                              [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))]])
# rotate the data using the rotation matrix
data = np.matmul(rot(theta), np.vstack([x, y]))

plt.scatter(data[0, :], data[1, :])
plt.title("Original data")
plt.axis('equal')
plt.show()

means = np.mean(data, axis=1) # calculate the means
# calculate the covariance matrix
cov = np.cov(data)
# Calculate the eigen values and eigen vectors of the covariance matrix
eval, evec = LA.eig(cov)

centerd_data = data - means[:, np.newaxis]
xmin, xmax, ymin, ymax = np.min(centerd_data[0,:]), np.max(centerd_data[0,:]), np.min(centerd_data[1,:]), np.max(centerd_data[1,:])

def getEigenAngle(v):
    print(v)
    return np.rad2deg(np.arctan(v[1]/v[0]))

def getGrippingAngle(eval, evec):
    # returns the angle facing against the shorter sides of the rectangle obb
    # the shorter side is the one with the smallest eigen value
    min_eigen_value_index = np.argmin(eval)
    return getEigenAngle(evec[:, min_eigen_value_index])

theta_pc1 = getEigenAngle(evec[:, 0]) # the eigen vectors are usually returned sorted based on their eigenvalues, 
# so we use the eigen vector in first column
gripping_angle = getGrippingAngle(eval, evec)
theta_pc1

aligned_coords = np.matmul(rot(-theta_pc1), centerd_data) # notice the minus theta_pc1 angle
xmin, xmax, ymin, ymax = np.min(aligned_coords[0, :]), np.max(aligned_coords[0, :]), np.min(aligned_coords[1, :]), np.max(aligned_coords[1, :])

# compute the minimums and maximums of each dimension again
rectCoords = lambda x1, y1, x2, y2: np.array([[x1, x2, x2, x1],
                                              [y1, y1, y2, y2]])

rectangleCoordinates = rectCoords(xmin, ymin, xmax, ymax)

# the side parallel to the gripping angle is the larger (due to eigen value comparison)
# the side perpendicular to the gripping angle is the smaller
parallel_side_length = np.max([xmax - xmin, ymax - ymin])
perpendicular_side_length = np.min([xmax - xmin, ymax - ymin])

centerCoordinates = [xmin + (xmax - xmin)/2, ymin + (ymax - ymin)/2]
rotateBack = np.matmul(rot(theta_pc1), aligned_coords) # notice the plus theta_pc1 angle
centerCoordinates = np.matmul(rot(theta_pc1), centerCoordinates)
rectangleCoordinates = np.matmul(rot(theta_pc1), rectangleCoordinates)

# translate back
rotateBack += means[:, np.newaxis]
rectangleCoordinates += means[:, np.newaxis]
plt.title("Re rotated and translated data")
plt.scatter(rotateBack[0, :], rotateBack[1, :])
# center
plt.plot(centerCoordinates[0], centerCoordinates[1], 'ro')
# four sides of the rectangle
plt.plot(rectangleCoordinates[0, 0:2], rectangleCoordinates[1, 0:2], color='r') # | (up)
plt.plot(rectangleCoordinates[0, 1:3], rectangleCoordinates[1, 1:3], color='g') # -->
plt.plot(rectangleCoordinates[0, 2:], rectangleCoordinates[1, 2:], color='b')    # | (down)
plt.plot([rectangleCoordinates[0, 3], rectangleCoordinates[0, 0]], [rectangleCoordinates[1, 3], rectangleCoordinates[1, 0]], color='k')    # <--

angle = gripping_angle
# draw an arrow aiming at angle +90
plt.arrow(centerCoordinates[0], centerCoordinates[1], 2*np.cos(np.deg2rad(angle + 90)), 2*np.sin(np.deg2rad(angle + 90)), head_width=0.2, head_length=1, fc='r', ec='r')

print("Center of the rectangle: ", centerCoordinates)
print("Rectangle coordinates: ", rectangleCoordinates)
print("Parallel (longer) side length: ", parallel_side_length)
print("Perpendicular (shorter) side length: ", perpendicular_side_length)
print("Obb Angle to grip: ", angle)

# same limits from y and x axis (same scale)
plt.axis('equal')

plt.show()