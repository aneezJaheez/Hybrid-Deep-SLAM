import torch
import numpy as np
import cv2 as cv
#from matplotlib import pyplot as plt


class Deproject():
    def __init__(self, calib_file, num_disparities, block_size, window_size, use_velodyne=False, calib_file_velo="/home/aneezahm001/Desktop/slam/data/kitti_mot/calib/calib_velo_to_cam.txt"):
        self.num_disparities = num_disparities
        self.block_size = block_size
        self.window_size = window_size
        self.use_velodyne=use_velodyne

        if use_velodyne:
            self.R_cam_velo, self.T_cam_velo = self.read_calib_file_velo(calib_file_velo)
        
        self.P_left, self.P_right, self.R_left_rect, self.R_right_rect = self.read_calib_file(calib_file)
        self.K_left, self.R_left, self.T_left = self.decompose_projection_matrix(self.P_left)
        self.K_right, self.R_right, self.T_right = self.decompose_projection_matrix(self.P_right)

        self.depth_map = None
        self.disparity = None
        self.velo_camera = None

    def read_calib_file_velo(self, path):
        with open(path, 'r') as f:
            calib = f.readlines()

        R_cam_velo = np.array([float(x) for x in calib[1].strip().split(' ')[1:]]).reshape((3, 3))
        t_cam_velo = np.array([float(x) for x in calib[2].strip().split(' ')[1:]])[:, None]

        T_cam_velo = np.vstack((np.hstack((R_cam_velo, t_cam_velo)),
                                np.array([0, 0, 0, 1])))
        
        return R_cam_velo, T_cam_velo

    def read_calib_file(self, path):
        with open(path) as f:
            calib = f.readlines()

        P_left = np.array([float(x) for x in calib[25].strip().split(' ')[1:]]).reshape((3,4))
        P_right = np.array([float(x) for x in calib[33].strip().split(' ')[1:]]).reshape((3,4))

        # get rectified rotation matrices
        R_left_rect = np.array([float(x) for x in calib[24].strip().split(' ')[1:]]).reshape((3, 3,))
        R_right_rect = np.array([float(x) for x in calib[32].strip().split(' ')[1:]]).reshape((3, 3,))

        R_left_rect = np.insert(R_left_rect, 3, values=[0,0,0], axis=0)
        R_left_rect = np.insert(R_left_rect, 3, values=[0,0,0,1], axis=1)

        return P_left, P_right, R_left_rect, R_right_rect

    def decompose_projection_matrix(self, P):    
        K, R, T, _, _, _, _ = cv.decomposeProjectionMatrix(P)
        T = T/T[3]

        return K, R, T
    
    def update_velo_camera(self, lidar_path, left_image):
        self.velo_camera = self.project_velo2cam(lidar_path, left_image)
    
    def compute_sgbm_disparity(self, left_image, right_image, display=False):
        """ Computes the disparity of an image pair using the SGBM algoithm.
            Inputs: 
                image_left/_right - (MxN) grayscale input images
                see opencv documentation for "StereoBM_create"
            Outputs:
                disparity (MxN) computed disparity map for the input images
            
            NOTE: image_left must be the left image (same for the right) or 
                unexpected results will occur due to 
        """
        # P1 and P2 control disparity smoothness (recommended values below)
        P1 = 8 * 3 * self.window_size**2
        P2 = 32 * 3 * self.window_size**2
        sgbm_obj = cv.StereoSGBM_create(0, self.num_disparities, self.block_size, 
            P1, P2, mode=cv.STEREO_SGBM_MODE_SGBM_3WAY)
            
        # compute disparity
        disparity = sgbm_obj.compute(left_image, right_image).astype(np.float32)/16.0

        # if display:
        # plt.figure(figsize = (40,20))
        # plt.imshow(disparity, cmap='cividis')
        # plt.title('Disparity Map', size=25)
        # plt.show();

        self.disparity = disparity

    def update_depth_map(self):
        self.calc_depth_map()

    def calc_depth_map(self):
        ''' Computes Depth map from Intrinsic Camera Matrix and Translations vectors.
            For KITTI, the depth is in meters.
            '''
        # Get the focal length from the K matrix
        f = self.K_left[0, 0]
        
        # Get the distance between the cameras from the t matrices (baseline)
        b = np.abs(self.T_left[0] - self.T_right[0])[0]
        
        # Replace all instances of 0 and -1 disparity with a small minimum value (to avoid div by 0 or negatives)
        self.disparity[self.disparity <= 0] = 1e-5
        
        # Calculate the depths 
        depth_map = f*b / self.disparity

        self.depth_map = depth_map

    def deproject_2d3d(self, bbox, method="median", draw=False):
        pt1 = np.asarray(bbox[0:2], dtype=np.int64)
        pt2 = np.asarray(bbox[2:4], dtype=np.int64)

        # get center location on image
        x_center = np.round((pt1[1] + pt2[1]) / 2).astype(int)
        y_center = np.round((pt1[0] + pt2[0]) / 2).astype(int)

        # get depth slice
        depth_slice = self.depth_map[pt1[1]:pt2[1], pt1[0]:pt2[0]]

        # compute median depth to get the distance
        if method == 'center':
            x_c = np.round((pt2[1] - pt1[1]) / 2).astype(int)
            y_c = np.round((pt2[0] - pt1[0]) / 2).astype(int)
            stereo_depth = depth_slice[x_c, y_c]
        else:
            stereo_depth = np.median(depth_slice)

        #point = np.array([x_center, y_center])
        point = np.array([y_center, x_center])
        point_3d_homogeneous = np.append(point, [1])
        point_3d = np.linalg.inv(self.K_left).dot(point_3d_homogeneous)
        #point_3d_norm = point_3d / point_3d[2]
        cam_coordinates = stereo_depth * point_3d
        return cam_coordinates
    
    def deproject_velodyne(self, bbox, method="median"):
        u, v, z = self.velo_camera
        
        pt1 = np.asarray(bbox[0:2], dtype=np.int64)
        pt2 = np.asarray(bbox[2:4], dtype=np.int64)
        
        # get center location
        x_center = np.round((pt1[1] + pt2[1]) / 2).astype(int)
        y_center = np.round((pt1[0] + pt2[0]) / 2).astype(int)

        # now get the closest LiDAR points to the center
        center_delta = np.abs(np.array((v, u)) 
                      - np.array([[x_center, y_center]]).T)
        
        # choose coordinate pair with the smallest L2 norm
        min_loc = np.argmin(np.linalg.norm(center_delta, axis=0))

        velo_depth = z[min_loc]

        point = np.array([y_center, x_center])
        point_3d_homogeneous = np.append(point, [1])
        point_3d = np.linalg.inv(self.K_left).dot(point_3d_homogeneous)
        point_3d_norm = point_3d / point_3d[2]
        cam_coordinates = velo_depth * point_3d_norm
        return cam_coordinates

        # # get correpsonding LiDAR Centers
        # velo_center = np.array([v[min_loc], u[min_loc]])

        # return velo_depth, velo_center
    
    def get_velo2cam(self, lidar_bin):
        ''' Converts the LiDAR point cloud to camera (u, v, z) image coordinates, 
            where z is in meters
            '''
        # read in LiDAR data
        scan_data = np.fromfile(lidar_bin, dtype=np.float32).reshape((-1,4))

        # convert to homogeneous coordinate system
        velo_points = scan_data[:, 0:3] # (x, y, z) --> (front, left, up)
        velo_points = np.insert(velo_points, 3, 1, axis=1).T # homogeneous LiDAR points

        # delete negative liDAR points
        velo_points = np.delete(velo_points, np.where(velo_points[3, :] < 0), axis=1) 

        # possibly use RANSAC to remove the ground plane for better viewing?

        # convert to camera coordinates
        velo_camera = self.P_left @ self.R_left_rect @ self.T_cam_velo @ velo_points

        # delete negative camera points ??
        velo_camera  = np.delete(velo_camera , np.where(velo_camera [2,:] < 0)[0], axis=1) 

        # get camera coordinates u,v,z
        velo_camera[:2] /= velo_camera[2, :]

        return velo_camera
    
    def project_velo2cam(self, lidar_bin, image):
        ''' Projects LiDAR point cloud onto the image coordinate frame '''

        # get camera (u, v, z) coordinates
        velo_camera = self.get_velo2cam(lidar_bin)

        (u, v, z) = velo_camera

        # remove outliers (points outside of the image frame)
        img_h, img_w, _ = image.shape
        u_out = np.logical_or(u < 0, u > img_w)
        v_out = np.logical_or(v < 0, v > img_h)
        outlier = np.logical_or(u_out, v_out)
        velo_camera = np.delete(velo_camera, np.where(outlier), axis=1)
        
        return velo_camera

