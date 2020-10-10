# CSCI 3302: Homework 3 -- Clustering and Classification
# Implementations of K-Means clustering and K-Nearest Neighbor classification
import pickle
import random
import copy
import heapq
import pdb
import matplotlib.pyplot as plt
from hw3_data import *
import numpy as np

# DONE: INSERT YOUR NAME HERE
LAST_NAME = "Simpson"


def visualize_data(data, cluster_centers_file):
  fig = plt.figure(1, figsize=(4,3))
  f = open(cluster_centers_file, 'rb')
  centers = pickle.load(f)
  f.close()

  km = KMeansClassifier()
  km._cluster_centers = centers

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

  labels = []
  center_colors = []
  for pt in data:
    labels.append(colors[km.classify(pt) % len(colors)])

  for i in range(len(centers)):
    center_colors.append(colors[i])

  plt.scatter([d[0] for d in data], [d[1] for d in data], c=labels, marker='x')
  plt.scatter([c[0] for c in centers], [c[1] for c in centers], c=center_colors, marker='D')
  plt.title("K-Means Visualization")
  plt.show()


class KMeansClassifier(object):

    def __init__(self):
        self._cluster_centers = [] # List of cluster centers, each of which is a point. ex: [ [10,10], [2,1], [0,-3] ]
        self._data = [] # List of datapoints (list of immutable lists, ex:  [ (0,0), (1.,5.), (2., 3.) ] )

    def add_datapoint(self, datapoint):
        self._data.append(datapoint)

    def fit(self, k):
        # Fit k clusters to the data, by starting with k randomly selected cluster centers.
        self._cluster_centers = [] # Reset cluster centers array
        cluster_points = {} #Dictionary for holding each centers points

        # DONE: Initialize k cluster centers at random points
        # HINT: To choose reasonable initial cluster centers, you can set them to be in the same spot as random (different) points from the dataset
        # Following k++ algorithim for choosing point here
        
        last_center = random.choice(self._data)
        self._cluster_centers.append(last_center)

        for i in range(1, k):
            max_dist = 0
            for p in self._data:
                dist = np.sqrt((last_center[0] - p[0])**2 + (last_center[1] - p[1])**2)
                if dist > max_dist:
                    center = p
            try:
                self._cluster_centers.append(center)
                last_center = center
            except:
                self._cluster_centers.append(random.choice(self._data))

        # for i in range(k):
        #     choice = random.choice(self._data)
        #     while choice in self._cluster_centers:
        #         choice = random.choice(self._data)
        #
        #     self._cluster_centers.append(choice)


        # TODO Follow convergence procedure to find final locations for each center
        while True:
            for i in range(k):
                cluster_points[i] = []

            cluster_centers_new = [] #List for holding new centers
            # DONE: Iterate through each datapoint in self._data and figure out which cluster it belongs to
            # HINT: Use self.classify(p) for each datapoint p
            for p in self._data:
                center = self.classify(p)
                cluster_points[center].append(p)

            # DONE: Figure out new positions for each cluster center (should be the average position of all its points)
            for c in cluster_points.keys():
                sum = [0,0]
                for p in cluster_points[c]:
                    sum[0] += p[0]
                    sum[1] += p[1]

                #Averaging
                num_points = len(cluster_points[c])
                try:
                    sum[0] /= num_points
                    sum[1] /= num_points
                    cluster_centers_new.insert(c,sum)
                except:
                    cluster_centers_new.insert(c,self._cluster_centers[c])

            # DONE: Check to see how much the cluster centers have moved (for the stopping condition)
            error = 0
            for i in range(k):
                old_center = self._cluster_centers[i]
                new_center = cluster_centers_new[i]
                error += np.sqrt((new_center[0] - old_center[0])**2 + (new_center[1] - old_center[1])**2)

            print(error)
            self._cluster_centers = copy.copy(cluster_centers_new)

            if error == 0: # DONE: If the centers have moved less than some predefined threshold (you choose!) then exit the loop
                break

            # TODO: Add each of the 'k' final cluster_centers to the model (self._cluster_centers)


    def classify(self,p):
        # Given a data point p, figure out wcluster_centers_newhich cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
        closest_cluster_index = 0

        # DONE: Find nearest cluster center, then return its index in self._cluster_centers
        center = self._cluster_centers[0]
        dist = np.sqrt((center[0] - p[0])**2 + (center[1] - p[1])**2)

        for i in range(1, len(self._cluster_centers)):
            center = self._cluster_centers[i]
            new_dist = np.sqrt((center[0] - p[0])**2 + (center[1] - p[1])**2)

            if new_dist < dist:
                closest_cluster_index = i
                dist = new_dist

        return closest_cluster_index

class Point(object):

    def __init__(self, point, distance):
        self.point = point[0]
        self.label = point[1]
        self.distance = distance

    def __lt__(self, other):
        return self.distance < other.distance

class KNNClassifier(object):

    def __init__(self):
        self._data = [] # list of (datapoint, label) tuples

    def clear_data(self):
        # Removes all data stored within the model
        self._data = []

    def add_labeled_datapoint(self, data_point, label):
        # Adds a labeled datapoint tuple onto the object's _data member
        self._data.append((data_point, label))

    def classify_datapoint(self, data_point, k):
        label_counts = {} # Dictionary mapping "label" => vote count
        best_label = None
        heap = [] #Heap for holding nearest neighbors

        # Perform k_nearest_neighbor classification, setting best_label to the majority-vote label for k-nearest points
        #DONE: Find the k nearest points in self._data to data_point
        for p in self._data:
            dist = np.sqrt((data_point[0] - p[0][0])**2 + (data_point[1] - p[0][1])**2)
            pt = Point(p,dist)
            heap.append(pt)

        heapq.heapify(heap)

        #DONE: Populate label_counts with the number of votes each label got from the k nearest points
        for i in range(k):
            pt = heapq.heappop(heap)
            dist = pt.distance
            label = pt.label
            try:
                if dist != 0:
                    label_counts[label] += 1.0/dist
            except:
                if dist != 0:
                    label_counts.update( {label : 1.0/dist} )
        #DONE: Make sure to scale the weight of the vote each point gets by how far away it is from data_point
        #      Since you're just taking the max at the end of the algorithm, these do not need to be normalized in any way
        best_label = max(label_counts.keys(), key=(lambda k: label_counts[k]))


        return best_label



def print_and_save_cluster_centers(classifier, filename):
  for idx, center in enumerate(classifier._cluster_centers):
    print("  Cluster %d, center at: %s" % (idx, str(center)))


  f = open(filename,'wb')
  pickle.dump(classifier._cluster_centers, f)
  f.close()

def read_data_file(filename):
  f = open(filename)
  data_dict = pickle.load(f)
  f.close()

  return data_dict['data'], data_dict['labels']

def read_hw_data():
  global hw_data
  data_dict = pickle.loads(hw_data)
  return data_dict['data'], data_dict['labels']

def main():
  global LAST_NAME
  # read data file
  #data, labels = read_data_file('hw3_data.pkl')

  # load dataset
  data, labels = read_hw_data()

  # data is an 'N' x 'M' matrix, where N=number of examples and M=number of dimensions per example
  # data[0] retrieves the 0th example, a list with 'M' elements, one for each dimension (xy-points would have M=2)
  # labels is an 'N'-element list, where labels[0] is the label for the datapoint at data[0]


  ########## PART 1 ############
  # perform K-means clustering
  kMeans_classifier = KMeansClassifier()
  for datapoint in data:
    kMeans_classifier.add_datapoint(datapoint) # add data to the model

  kMeans_classifier.fit(4) # Fit 4 clusters to the data

  # plot results
  print('\n'*2)
  print("K-means Classifier Test")
  print('-'*40)
  print("Cluster center locations:")
  print_and_save_cluster_centers(kMeans_classifier, "hw3_kmeans_" + LAST_NAME + ".pkl")

  print('\n'*2)


  ########## PART 2 ############
  print("K-Nearest Neighbor Classifier Test")
  print('-'*40)

  # Create and test K-nearest neighbor classifier
  kNN_classifier = KNNClassifier()
  k = 3
  debug = False

  correct_classifications = 0
  # Perform leave-one-out cross validation (LOOCV) to evaluate KNN performance
  for holdout_idx in range(len(data)):
    # Reset classifier
    kNN_classifier.clear_data()

    for idx in range(len(data)):
      if idx == holdout_idx: continue # Skip held-out data point being classified

      # Add (data point, label) tuples to KNNClassifier
      kNN_classifier.add_labeled_datapoint(data[idx], labels[idx])

    guess = kNN_classifier.classify_datapoint(data[holdout_idx], k) # Perform kNN classification

    #Debugging
    if debug == True:
        print(guess,labels[holdout_idx])

    if guess == labels[holdout_idx]:
      correct_classifications += 1.0

  print("kNN classifier for k=%d" % k)
  print("Accuracy: %g" % (correct_classifications / len(data)))
  print('\n'*2)

  visualize_data(data, 'hw3_kmeans_' + LAST_NAME + '.pkl')


if __name__ == '__main__':
  main()
