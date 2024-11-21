#!/usr/bin/env python3
"""
This script provides a ROS service that receives a pointcloud and runs k-means clustering on it.
It returns the center x, y of the largest cluster.
"""

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import KMeans
from object_detector_3d.srv import Clustering
import matplotlib.pyplot as plt
import cv2
import os
import rospkg

ARGS= {
    "CLUSTERS_PER_OBJECT": 3,
    "SAVE_IMAGE": True
}

class Clustering_Service:
    def __init__(self):
        """
        Initialize the Clustering_Service class.
        """
        self.service = rospy.Service('Clustering', Clustering, self.handle_clustering)
        self.clusters_per_object = ARGS["CLUSTERS_PER_OBJECT"]
        self.save_image = ARGS["SAVE_IMAGE"]
        rospy.loginfo("Clustering service ready")
        rospy.spin()
    
    def handle_clustering(self, req):
        """
        Handle the Clustering service request.
        """
        point_cloud = req.pointcloud
        point_cloud_array = []
        for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                point_cloud_array.append([p[0], p[1]])
        point_cloud_array = np.array(point_cloud_array)

        n_clusters = req.n_clusters * self.clusters_per_object if req.n_clusters > 0 else 1
        rospy.loginfo("Running k-means clustering with %d clusters", n_clusters)
        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(point_cloud_array)
        rospy.loginfo("Clustering complete, computing largest cluster")

        biggest_cluster = 0
        biggest_cluster_size = 0
        for i in range(n_clusters):
            if len(kmeans.labels_[kmeans.labels_ == i]) > biggest_cluster_size:
                biggest_cluster = i
                biggest_cluster_size = len(kmeans.labels_[kmeans.labels_ == i])
        centroid = kmeans.cluster_centers_[biggest_cluster]
        rospy.loginfo("Largest cluster found, centroid: %s", centroid)

        if self.save_image:
            plt.scatter(point_cloud_array[:,0], point_cloud_array[:,1], c=kmeans.labels_, cmap='rainbow')
            plt.scatter(centroid[0], centroid[1], c='green', s=1000, alpha=0.8)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('Clusters')
            plt.axis('equal')

            rospy.loginfo("Saving clusters as image")
            folder = os.path.join(package_path, "images")
            if not os.path.exists(folder):
                os.makedirs(folder)
            plt.savefig(os.path.join(folder, 'clusters.png'))

        rospy.loginfo("Returning centroid")
        return centroid[0], centroid[1], True

def main():
    """
    Main function to initialize the Clustering_Service node.
    """
    rospy.init_node('Clustering_Service', anonymous=True)
    for key in ARGS:
        ARGS[key] = rospy.get_param('~' + key, ARGS[key])
    Clustering_Service()

if __name__ == '__main__':
    main()

# Examples and use cases for key technologies used

# Example of using rospy to create a ROS service for clustering
def example_rospy_service():
    rospy.init_node('example_clustering_service')
    service = rospy.Service('example_clustering_service', Clustering, handle_example_clustering_service)
    rospy.spin()

def handle_example_clustering_service(req):
    print("Handling example clustering service request")
    return ClusteringResponse(0.0, 0.0, True)

# Example of using numpy to process point cloud data
def example_numpy_processing():
    point_cloud_array = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    print("Point cloud array:", point_cloud_array)

# Example of using sklearn to perform k-means clustering
def example_sklearn_clustering():
    point_cloud_array = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    kmeans = KMeans(n_clusters=2, random_state=0).fit(point_cloud_array)
    print("Cluster centers:", kmeans.cluster_centers_)

# Example of using matplotlib to visualize point clouds
def example_matplotlib_visualization():
    point_cloud_array = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    plt.scatter(point_cloud_array[:,0], point_cloud_array[:,1], c='blue')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Point Cloud')
    plt.show()
