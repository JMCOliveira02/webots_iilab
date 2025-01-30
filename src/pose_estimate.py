import rospy
#from PointNet import *
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
import cv2
from sklearn.neighbors import KDTree

loaded_descriptors = np.load('/home/joao/catkin_ws/src/webots_iilab/src/descriptors_10000_2048_collab.npy')

print("Descriptors loaded successfully.")

print("Type of loaded_descriptors:", type(loaded_descriptors))
print("Length of loaded_descriptors:", len(loaded_descriptors))

if len(loaded_descriptors) > 0:
    print("Type of first element:", type(loaded_descriptors[0]))
    print("First element sample:", loaded_descriptors[0])

kd_tree = KDTree(loaded_descriptors)

poses = np.load('/home/joao/catkin_ws/src/webots_iilab/src/poses.npy')
# Initialize the global position variables
current_x, current_y = 0.0, 0.0

# Load your map image
map_image_path = "/home/joao/catkin_ws/src/webots_iilab/src/iilab.jpg"  # Replace with the path to your map image
map_image = cv2.imread(map_image_path)

if map_image is None:
    rospy.logerr(f"Failed to load image at {map_image_path}")
else:
    map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2RGB)  # Convert for matplotlib display
    map_height, map_width, _ = map_image.shape

map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2RGB)  # Convert for matplotlib display
map_height, map_width, _ = map_image.shape

num_points = 10000
num_features = 2048
# ============================================================================
# T-net (Spatial Transformer Network)
class Tnet(nn.Module):
    ''' T-Net learns a Transformation matrix with a specified dimension '''
    def __init__(self, dim, num_points=num_points):
        super(Tnet, self).__init__()

        # dimensions for transform matrix
        self.dim = dim

        self.conv1 = nn.Conv1d(dim, 64, kernel_size=1)
        self.conv2 = nn.Conv1d(64, 128, kernel_size=1)
        self.conv3 = nn.Conv1d(128, 1024, kernel_size=1)

        self.linear1 = nn.Linear(1024, 512)
        self.linear2 = nn.Linear(512, 256)
        self.linear3 = nn.Linear(256, dim**2)

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

        self.max_pool = nn.MaxPool1d(kernel_size=num_points)


    def forward(self, x):
        bs = x.shape[0]

        # pass through shared MLP layers (conv1d)
        x = self.bn1(F.relu(self.conv1(x)))
        x = self.bn2(F.relu(self.conv2(x)))
        x = self.bn3(F.relu(self.conv3(x)))

        # max pool over num points
        x = self.max_pool(x).view(bs, -1)

        # pass through MLP
        x = self.bn4(F.relu(self.linear1(x)))
        x = self.bn5(F.relu(self.linear2(x)))
        x = self.linear3(x)

        # initialize identity matrix
        iden = torch.eye(self.dim, requires_grad=True).repeat(bs, 1, 1)
        if x.is_cuda:
            iden = iden.cuda()

        x = x.view(-1, self.dim, self.dim) + iden

        return x

# ============================================================================
# Point Net Backbone (main Architecture)
class PointNetBackbone(nn.Module):
    '''
    This is the main portion of Point Net before the classification and segmentation heads.
    The main function of this network is to obtain the local and global point features,
    which can then be passed to each of the heads to perform either classification or
    segmentation. The forward pass through the backbone includes both T-nets and their
    transformations, the shared MLPs, and the max pool layer to obtain the global features.

    The forward function either returns the global or combined (local and global features)
    along with the critical point index locations and the feature transformation matrix. The
    feature transformation matrix is used for a regularization term that will help it become
    orthogonal. (i.e. a rigid body transformation is an orthogonal transform and we would like
    to maintain orthogonality in high dimensional space). "An orthogonal transformations preserves
    the lengths of vectors and angles between them"
    '''
    def __init__(self, num_points=num_points, num_global_feats=num_features, local_feat=True):
        ''' Initializers:
                num_points - number of points in point cloud
                num_global_feats - number of Global Features for the main
                                   Max Pooling layer
                local_feat - if True, forward() returns the concatenation
                             of the local and global features
            '''
        super(PointNetBackbone, self).__init__()

        # if true concat local and global features
        self.num_points = num_points
        self.num_global_feats = num_global_feats
        self.local_feat = local_feat

        # Spatial Transformer Networks (T-nets)
        self.tnet1 = Tnet(dim=3, num_points=num_points)
        self.tnet2 = Tnet(dim=64, num_points=num_points)

        # shared MLP 1
        self.conv1 = nn.Conv1d(3, 64, kernel_size=1)
        self.conv2 = nn.Conv1d(64, 64, kernel_size=1)

        # shared MLP 2
        self.conv3 = nn.Conv1d(64, 64, kernel_size=1)
        self.conv4 = nn.Conv1d(64, 128, kernel_size=1)
        self.conv5 = nn.Conv1d(128, self.num_global_feats, kernel_size=1)

        # batch norms for both shared MLPs
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(64)
        self.bn3 = nn.BatchNorm1d(64)
        self.bn4 = nn.BatchNorm1d(128)
        self.bn5 = nn.BatchNorm1d(self.num_global_feats)

        # max pool to get the global features
        self.max_pool = nn.MaxPool1d(kernel_size=num_points, return_indices=True)


    def forward(self, x):

        # get batch size
        bs = x.shape[0]

        # pass through first Tnet to get transform matrix
        A_input = self.tnet1(x)

        # perform first transformation across each point in the batch
        x = torch.bmm(x.transpose(2, 1), A_input).transpose(2, 1)

        # pass through first shared MLP
        x = self.bn1(F.relu(self.conv1(x)))
        x = self.bn2(F.relu(self.conv2(x)))

        # get feature transform
        A_feat = self.tnet2(x)

        # perform second transformation across each (64 dim) feature in the batch
        x = torch.bmm(x.transpose(2, 1), A_feat).transpose(2, 1)

        # store local point features for segmentation head
        local_features = x.clone()

        # pass through second MLP
        x = self.bn3(F.relu(self.conv3(x)))
        x = self.bn4(F.relu(self.conv4(x)))
        x = self.bn5(F.relu(self.conv5(x)))

        # get global feature vector and critical indexes
        global_features, critical_indexes = self.max_pool(x)
        global_features = global_features.view(bs, -1)
        critical_indexes = critical_indexes.view(bs, -1)

        if self.local_feat:
            features = torch.cat((local_features,
                                  global_features.unsqueeze(-1).repeat(1, 1, self.num_points)),
                                  dim=1)

            return features, critical_indexes, A_feat

        else:
            return global_features, critical_indexes, A_feat


# 1. Instantiate PointNetBackbone
pointnet_backbone = PointNetBackbone(num_points=num_points, local_feat=False) # Use local_feat=False for global features only
pointnet_backbone.load_state_dict(torch.load("/home/joao/catkin_ws/src/webots_iilab/src/pointnet_backbone_weights_10000_2048.pth"))

def extract_features_open3d(points, num_points):
    global pointnet_backbone
    print(points.shape)
    print(points.dtype)
    # Check for NaN and Inf in the original points
    has_nan = np.isnan(points).any()
    has_inf = np.isinf(points).any()

    if has_nan or has_inf:
      # Handle NaN and Inf values
      # Option 1: Remove points with NaN or Inf
      valid_mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
      points = points[valid_mask]

    # Sample one every k points
    k = len(points) // num_points
    sampled_indices = np.arange(0, len(points), k)[:num_points]
    points = points[sampled_indices]

    points -= np.mean(points, axis=0)
    norm = np.linalg.norm(points, axis=1)
    points /= (np.max(norm) + 1e-8)  # Avoid division by zero


    # Convert to Torch Tensor
    points_tensor = torch.from_numpy(points).float().unsqueeze(0).transpose(2, 1)

    # Move to Device
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    pointnet_backbone = pointnet_backbone.to(device)
    points_tensor = points_tensor.to(device)

    # Extract Features
    with torch.no_grad():
        pointnet_backbone.eval()
        features, critical_indexes, A_feat = pointnet_backbone(points_tensor)
    return features, critical_indexes, A_feat



x1, x2, x3, x4, x5 = 0, 0, 0, 0, 0
y1, y2, y3, y4, y5 = 0, 0, 0, 0, 0

def pcl_callback(data):
    global x1, x2, x3, x4, x5, y1, y2, y3, y4, y5
    points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(data, field_names=("x", "y", "z"))], dtype=np.float64)
    # Print or process the NumPy array
    #rospy.loginfo(f"Point cloud shape and type: {points.shape}{points.dtype}")
    features, critical, A_feat = extract_features_open3d(points, num_points)
    # Query the KDTree with the extracted features
    # Print the poses and their corresponding probabilities

    features = features.cpu().numpy()
    rospy.loginfo(features)
    dist, nearest_indices = kd_tree.query(features, k=5) 
    rospy.loginfo(f"Nearest indices: {nearest_indices}")


    # Convert to arrays
    nearest_indices = np.array(nearest_indices)
    # Print results
    x1, y1 = poses[nearest_indices[0][0]][:2]
    x2, y2 = poses[nearest_indices[0][1]][:2]
    x3, y3 = poses[nearest_indices[0][2]][:2]
    x4, y4 = poses[nearest_indices[0][3]][:2]
    x5, y5 = poses[nearest_indices[0][4]][:2]
    rospy.loginfo(nearest_indices)



def tf_callback(data):
    global current_x, current_y

    if data.transforms:
        # Get the last transform
        current_x, current_y = 0.0, 0.0
        last_transform = data.transforms[-1]
        translation = last_transform.transform.translation
        current_x = translation.x
        current_y = translation.y
        #rospy.loginfo(f"Translation x: {current_x}, y: {current_y}")

def update_window(event):
    global current_x, current_y
    global x1, x2, x3, x4, x5, y1, y2, y3, y4, y5

    # Map boundaries
    min_x, max_x = -18.2, 22.0
    min_y, max_y = -12.9, 14.35

    if min_x <= current_x <= max_x and min_y <= current_y <= max_y:
        px = int((current_x - min_x) / (max_x - min_x) * map_width)
        py = int((current_y - min_y) / (max_y - min_y) * map_height)

        px1 = int((x1-min_x) / (max_x - min_x) * map_width)
        py1 = int((y1-min_y) / (max_y - min_y) * map_height)

        px2 = int((x2-min_x) / (max_x - min_x) * map_width)
        py2 = int((y2-min_y) / (max_y - min_y) * map_height)

        px3 = int((x3-min_x) / (max_x - min_x) * map_width)
        py3 = int((y3-min_y) / (max_y - min_y) * map_height)

        px4 = int((x4-min_x) / (max_x - min_x) * map_width)
        py4 = int((y4-min_y) / (max_y - min_y) * map_height)

        px5 = int((x5-min_x) / (max_x - min_x) * map_width)
        py5 = int((y5-min_y) / (max_y - min_y) * map_height)

        # Create a copy of the map image to draw on
        image_with_dot = map_image.copy()
        cv2.circle(image_with_dot, (px, map_height - py), radius=5, color=(0, 255, 0), thickness=-1)
        cv2.circle(image_with_dot, (px1, map_height - py1), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.circle(image_with_dot, (px2, map_height - py2), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.circle(image_with_dot, (px3, map_height - py3), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.circle(image_with_dot, (px4, map_height - py4), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.circle(image_with_dot, (px5, map_height - py5), radius=3, color=(0, 0, 255), thickness=-1)

        cv2.imshow('Robot Position on Map', image_with_dot)
        cv2.waitKey(1)
    else:
        rospy.logwarn("Coordinates are out of map bounds!")

if __name__ == '__main__':
    rospy.init_node('pose_estimator', anonymous=True)
    rospy.Subscriber('/pointcloud3D', PointCloud2, pcl_callback)
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    rospy.Timer(rospy.Duration(0.1), update_window)  # Update every 0.1 seconds

    rospy.spin()
