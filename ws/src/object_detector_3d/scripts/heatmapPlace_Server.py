import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import cv2
import rospy
import rospkg
from frida_manipulation_interfaces.srv import HeatmapPlace
import sensor_msgs.point_cloud2 as pc2
import os

# get package path
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('object_detector_3d')

ARGS= {
    "SAVE_IMAGE": True
}

class HeatmapServer:
    def __init__(self):
        # Create service 
        self.service = rospy.Service('HeatmapPlace', HeatmapPlace, self.handle_heatmap)
        self.save_image = ARGS["SAVE_IMAGE"]
        rospy.loginfo("Heatmap Place Service ready")
        rospy.spin()
        
    def handle_heatmap(self, req):
        # Convert pointcloud to numpy array, ignore z axis
        point_cloud = req.pointcloud
        #print(point_cloud)
        point_cloud_array = []
        for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            # append point to array
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                point_cloud_array.append([p[0], p[1]])
        cloud_points = np.array(point_cloud_array)
        #print(point_cloud_array)
        
        # size of each square length in the grid in meters
        grid_size = 0.02

        plt.scatter(cloud_points[:, 0], cloud_points[:, 1])

        x, y = [], []
        for point in cloud_points:
            x.append(int(point[0]*1000))
            y.append(int(point[1]*1000))
        grid_size_mm = int(grid_size*1000)
        hist_range = [range(min(x), max(x)+grid_size_mm, grid_size_mm), range(min(y), max(y)+grid_size_mm, grid_size_mm)]
        hist, xedges, yedges = np.histogram2d(x, y, bins=hist_range, range=[[min(x), max(x)], [min(y), max(y)]])

        plt.subplot(221)
        plt.scatter(x, y)
        plt.subplot(222)
        plt.imshow(hist.T, origin='lower', extent=[min(x), max(x), min(y), max(y)], aspect='auto')

        # binary map, 1 if the histogram value is greater than 0, 0 otherwise
        binary_map = hist > 0
        plt.subplot(223)
        plt.imshow(binary_map.T, origin='lower', extent=[min(x), max(x), min(y), max(y)], aspect='auto')

        binary_map_array = np.array(binary_map, dtype=bool)

        # each cell in the binary map with a value of 1 means there is table to place, so it adds heat
        # every cell in the binary map with a value of 0 means there is no table to place, so it is instantly cooled to 0

        # the heat map is generated with a kernel of nxn size, with a gaussian distribution
        heat_kernel_length = 0.4 # each free cell will heat up grid_size*kernel_length meters around it
        cool_kernel_length = 0.3 # each cell with table will cool down grid_size*kernel_length meters around it
        heat_multiplier = 1
        cool_multiplier = 1.5

        heat_kernel_size = int(heat_kernel_length/grid_size)
        heat_kernel = np.zeros((heat_kernel_size, heat_kernel_size))
        for i in range(heat_kernel_size):
            for j in range(heat_kernel_size):
                heat_kernel[i, j] = heat_multiplier* np.exp(-((i-heat_kernel_size//2)**2 + (j-heat_kernel_size//2)**2)/(2*heat_kernel_size**2))

        cool_kernel_size = int(cool_kernel_length/grid_size)
        cool_kernel = np.zeros((cool_kernel_size, cool_kernel_size))
        for i in range(cool_kernel_size):
            for j in range(cool_kernel_size):
                cool_kernel[i, j] = cool_multiplier* np.exp(-((i-cool_kernel_size//2)**2 + (j-cool_kernel_size//2)**2)/(2*cool_kernel_size**2))

        print(heat_kernel)
        print(cool_kernel)

        # first pass to heat with the table points
        heat_map = np.zeros_like(binary_map_array, dtype=float)
        cool_map = np.zeros_like(binary_map_array, dtype=float)
        for i in range(binary_map_array.shape[0]):
            for j in range(binary_map_array.shape[1]):
                # heat
                if binary_map_array[i, j] == 1:
                    for k in range(heat_kernel_size):
                        for l in range(heat_kernel_size):
                            if i-heat_kernel_size//2+k >= 0 and i-heat_kernel_size//2+k < binary_map_array.shape[0] and j-heat_kernel_size//2+l >= 0 and j-heat_kernel_size//2+l < binary_map_array.shape[1]:
                                heat_map[i-heat_kernel_size//2+k, j-heat_kernel_size//2+l] += heat_kernel[k, l]
                else:
                    for k in range(cool_kernel_size):
                        for l in range(cool_kernel_size):
                            if i-cool_kernel_size//2+k >= 0 and i-cool_kernel_size//2+k < binary_map_array.shape[0] and j-cool_kernel_size//2+l >= 0 and j-cool_kernel_size//2+l < binary_map_array.shape[1]:
                                cool_map[i-cool_kernel_size//2+k, j-cool_kernel_size//2+l] += cool_kernel[k, l]
        # show binary map array

        # shift all the values to be positive
        heat_map = heat_map - cool_map
        # anywhere there is no table, it is instantly cooled to 0
        heat_map[binary_map_array == 0] = 0
        heat_map[heat_map <= 0] = 0
        # normalize excluding the 0 values
        # min_heat = np.min(heat_map[heat_map != 0])
        # max_heat = np.max(heat_map)
        # heat_map = (heat_map - min_heat) / (max_heat - min_heat)
        heat_map[binary_map_array == 0] = 0


        plt.subplot(224)
        # plot the values of the heat map
        plt.imshow(heat_map.T, origin='lower', extent=[min(x), max(x), min(y), max(y)], aspect='auto', cmap='gnuplot2')

        # mark the hottest spot
        max_index = np.unravel_index(np.argmax(heat_map, axis=None), heat_map.shape)
        max_index_location = np.multiply(max_index, int(grid_size*1000))
        print(f"Shape: ", heat_map.shape)
        print(max_index)
        print(max_index_location)
        # mark with green cross
        plt.plot(max_index_location[0] + min(x) + int(grid_size_mm/2), max_index_location[1] + min(y) + int(grid_size_mm/2), 'g+', markersize=10)

        if not os.path.exists(package_path + '/heatmap_images'):
            os.makedirs(package_path + '/heatmap_images')
        
        if self.save_image:
            plt.savefig(package_path + '/heatmap_images/heatmap.png')
            
def main():
    rospy.init_node('Heatmap Place Service', anonymous=True)
    for key in ARGS:
        ARGS[key] = rospy.get_param('~' + key, ARGS[key])
    HeatmapPlace()

if __name__ == '__main__':
    main()
