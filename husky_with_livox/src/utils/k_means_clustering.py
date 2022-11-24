import numpy as np
import open3d as o3d
import copy
import random
from matplotlib import pyplot as plt
from sklearn import datasets
from mpl_toolkits.mplot3d import Axes3D


def draw_labels_on_model(pcl,labels):
    cmap = plt.get_cmap("tab20")
    pcl_temp = copy.deepcopy(pcl)
    max_label = labels.max()
    colors = cmap(labels / (max_label if max_label > 0 else 1))
    pcl_temp.colors = o3d.utility.Vector3dVector(colors[:,:3])
    o3d.visualization.draw_geometries([pcl_temp])

def create_point_cloud():
    d = 4
    mesh = o3d.geometry.TriangleMesh.create_tetrahedron().translate((-d, 0, 0))
    mesh += o3d.geometry.TriangleMesh.create_octahedron().translate((0, 0, 0))
    mesh += o3d.geometry.TriangleMesh.create_icosahedron().translate((d, 0, 0))
    mesh += o3d.geometry.TriangleMesh.create_torus().translate((-d, -d, 0))
    mesh += o3d.geometry.TriangleMesh.create_moebius(twists=1).translate((0, -d, 0))
    mesh += o3d.geometry.TriangleMesh.create_moebius(twists=2).translate((d, -d, 0))
    point_cloud = mesh.sample_points_uniformly(int(1e3))
    return point_cloud

def normalize(X,axis=-1,p=2):
    #normalize the array and then transfer into a vector
    lp_norm = np.atleast_1d(np.linalg.norm(X,p,axis))
    lp_norm[lp_norm == 0] = 1
    #expand a dimension along axis for lp_norm
    #this is to make sure X and lp_norm have the same dimensions
    return X / np.expand_dims(lp_norm,axis)

def euclidean_distance(one_sample,X):
    #transfer one_sample into 1D vector
    one_sample = one_sample.reshape(1,-1)
    #transfer X into 1D vector
    X = X.reshape(X.shape[0],-1)
    #this is used to make sure one_sample's dimension is same as X
    distances = np.power(np.tile(one_sample,(X.shape[0],1))-X,2).sum(axis=1)
    return distances

class KMeans():
    def __init__(self,
                 k = 2,
                 max_iterations = 1500,
                 tolerance = 0.00001):
        self.k = k
        self.max_iterations = max_iterations
        self.tolerance = tolerance
    
    #randomly select k centroids
    def init_random_centroids(self, X):
        #save the shape of X
        n_samples, n_features = np.shape(X)
        #make a zero matrix to store values
        centroids = np.zeros((self.k,n_features))
        #bcs there is k centroids, so we loop k tiems
        for i in range(self.k):
            #selecting values under the range radomly
            centroid = X[np.random.choice(range(n_samples))]
            centroids[i] = centroid
        return centroids

    def closest_centroid(self,sample,centroids):
        """ 
        Finds the closest centroid of a sample.
        """
        distances = euclidean_distance(sample,centroids)
        #np.argmin return the indices of the minimum of distances
        closest_i = np.argmin(distances)
        return closest_i

    def create_clusters(self,centroids, X):
        """
        Determine the clusters.
        """
        n_samples = np.shape(X)[0]
        #This is to construct the nested list for storing clusters
        clusters = [[] for _ in range(self.k)]
        for sample_i, sample in enumerate(X):
            centroid_i = self.closest_centroid(sample,centroids)
            clusters[centroid_i].append(sample_i)
        return clusters

    def update_centroids(self,clusters,X):
        """
        Updates the centroids based on mean algorithm.
        """
        n_features = np.shape(X)[1]
        centroids = np.zeros((self.k,n_features))
        for i, cluster in enumerate(clusters):
            centroid = np.mean(X[cluster],axis=0)
            centroids[i] = centroid
        return centroids

    def get_cluster_labels(self,clusters,X):
        """
        Obtains the labels
        same cluster, same y_pred value
        """
        y_pred = np.zeros(np.shape(X)[0])
        for cluster_i, cluster in enumerate(clusters):
            for sample_i in cluster:
                y_pred[sample_i] = cluster_i
        return y_pred

    def predict(self, X):
        """
        Predicts the labels.       
        """
        #selecting the centroids randomly
        centroids = self.init_random_centroids(X)

        for _ in range(self.max_iterations):
            #clustering all the data point
            clusters = self.create_clusters(centroids,X)
            former_centroids = centroids
            #calculate new cluster center
            centroids = self.update_centroids(clusters,X)
            #judge the current difference if it meets convergence  
            diff = centroids - former_centroids
            if diff.any() < self.tolerance:
                break
            
        return self.get_cluster_labels(clusters,X) 

if __name__ == "__main__":

    point_cloud = create_point_cloud()
    xyz_pc = np.asarray(point_cloud.points)
    clf = KMeans(k = 6)
    labels = clf.predict(xyz_pc)
    
    draw_labels_on_model(point_cloud,labels)