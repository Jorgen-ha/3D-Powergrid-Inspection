import numpy as np
import open3d as o3d
import copy

def draw_registrations(source, target, transformation = None, recolor = False):
    """Function to visualise two pointclouds, and optionally a given transformation.
    Uses open3d to visualise, and colours the point clouds differently if indicated.

    Args:
        source (open3d.pcd): the source point cloud
        target (open3d.pcd): the target point cloud
        transformation (np.array, optional): a 4x4 transformation matrix. Defaults to None.
        recolor (bool, optional): colours the point clouds in two different colours. Defaults to False.
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if(recolor):
        source_temp.paint_uniform_color([1, 0.706, 0])     # orange
        target_temp.paint_uniform_color([0, 0.651, 0.929]) # blue
    if(transformation is not None):
        source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def find_proper_voxel_size(pcd, target_points=800, step=0.05):
    """Function finding the appropiate voxel size to give a target pcd with 
    less than 800 points (seems to give good transformations consistently). 

    Args:
        pcd (open3d.pcd): point cloud to downsample
        target_points (int, optional): target number of points. Defaults to 800.
        step (float, optional): step size to increase the voxel size. Defaults to 0.05.
    
    Returns:
        float: voxel size
    """
    tmp_pcd = copy.deepcopy(pcd)
    n_points = len(np.asarray(tmp_pcd.points))
    voxel_size = 0.0
    while n_points > target_points:
        voxel_size = voxel_size + step
        target_down = tmp_pcd.voxel_down_sample(voxel_size)
        n_points = len(np.asarray(target_down.points))
    
    return voxel_size

def calculate_largest_distance(pcd):
    """Function calculating the biggest difference between any two points in the provided point cloud

    Args:
        pcd (open3d.pcd): the point cloud
    
    Returns:
        float: the largest distance between any two points
    """
    
    points = np.array(pcd.points)
    
    # Calculate pairwise distances efficiently using vectorization
    pairwise_distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)

    # Set diagonal elements (distances to itself) to zero
    pairwise_distances[np.arange(len(points))[:, None] == np.arange(len(points))] = 0

    # Find the maximum distance
    largest_distance = pairwise_distances.max()

    return largest_distance

def calculate_scale(source, target):
    """Function calculating the scale difference between two point clouds, assuming
    the two point clouds are of the same object. We calculate the scale for the source
    point cloud to be multiplied with, making it match the target point cloud.

    Args:
        source (open3d.pcd): the source point cloud
        target (open3d.pcd): the target point cloud
    
    Returns:
        float: the scale between the two point clouds
    """
    largest_distance_source = calculate_largest_distance(source)
    largest_distance_target = calculate_largest_distance(target)

    return largest_distance_target / largest_distance_source

def find_pcd_height(pcd, visualise=False):
    """Finds the height of the point cloud by looking at the min and max z values.

    Args:
        pcd (open3d.pcd): the point cloud to find the height of
        visualise (bool, optional): whether or not to visualise the height. Defaults to False.

    Returns:
        float: the height of the point cloud
    """
    min_z_idx = np.argmin(np.asarray(pcd.points)[:,2])
    max_z_idx = np.argmax(np.asarray(pcd.points)[:,2])
    min_z = np.asarray(pcd.points)[min_z_idx]
    max_z = np.asarray(pcd.points)[max_z_idx]
    height = max_z[2] - min_z[2]
    
    if visualise:
        min_z_pcd = o3d.geometry.PointCloud()
        min_z_pcd.points = o3d.utility.Vector3dVector([min_z])
        min_z_pcd.paint_uniform_color([1, 0, 0])
        
        max_z_pcd = o3d.geometry.PointCloud()
        max_z_pcd.points = o3d.utility.Vector3dVector([max_z])
        max_z_pcd.paint_uniform_color([0, 1, 0])
        
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector([min_z, max_z])
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        line.paint_uniform_color([0.9, 0.6, 0.7]) # pink
        
        print(f"Visualising the point cloud height, min point in red, max point in green and line between in pink.")
        o3d.visualization.draw_geometries([pcd, min_z_pcd, max_z_pcd, line])
    
    return height

def find_pcd_width(pcd, visualise=False):
    """Finds the width of the point cloud by looking at the min and max x values.

    Args:
        pcd (open3d.pcd): the point cloud to find the width of
        visualise (bool, optional): whether or not to visualise the width. Defaults to False.

    Returns:
        float: the width of the point cloud
    """
    min_x_idx = np.argmin(np.asarray(pcd.points)[:,0])
    max_x_idx = np.argmax(np.asarray(pcd.points)[:,0])
    min_x = np.asarray(pcd.points)[min_x_idx]
    max_x = np.asarray(pcd.points)[max_x_idx]
    
    width = max_x[0] - min_x[0]
    
    if visualise:
        min_x_pcd = o3d.geometry.PointCloud()
        min_x_pcd.points = o3d.utility.Vector3dVector([min_x])
        min_x_pcd.paint_uniform_color([1, 0, 0])
        
        max_x_pcd = o3d.geometry.PointCloud()
        max_x_pcd.points = o3d.utility.Vector3dVector([max_x])
        max_x_pcd.paint_uniform_color([0, 1, 0])
        
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector([min_x, max_x])
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        line.paint_uniform_color([0.9, 0.6, 0.7]) # pink
        
        print(f"Visualising the point cloud width, min point in red, max point in green and line between in pink.")
        o3d.visualization.draw_geometries([pcd, min_x_pcd, max_x_pcd, line])
    
    return width

def find_pcd_length(pcd, visualise=False):
    """Finds the length of the point cloud by looking at the min and max y values.

    Args:
        pcd (open3d.pcd): the point cloud to find the length of
        visualise (bool, optional): whether or not to visualise the length. Defaults to False.

    Returns:
        float: the length of the point cloud
    """
    min_y_idx = np.argmin(np.asarray(pcd.points)[:,2])
    max_y_idx = np.argmax(np.asarray(pcd.points)[:,2])
    min_y = np.asarray(pcd.points)[min_y_idx]
    max_y = np.asarray(pcd.points)[max_y_idx]
    length = max_y[2] - min_y[2]

    if visualise:
        min_y_pcd = o3d.geometry.PointCloud()
        min_y_pcd.points = o3d.utility.Vector3dVector([min_y])
        min_y_pcd.paint_uniform_color([1, 0, 0])
        
        max_y_pcd = o3d.geometry.PointCloud()
        max_y_pcd.points = o3d.utility.Vector3dVector([max_y])
        max_y_pcd.paint_uniform_color([0, 1, 0])
        
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector([min_y, max_y])
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        line.paint_uniform_color([0.9, 0.6, 0.7]) # pink
        
        print(f"Visualising the point cloud length, min point in red, max point in green and line between in pink.")
        o3d.visualization.draw_geometries([pcd, min_y_pcd, max_y_pcd, line])

    return length

def perform_fgr(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """Function performing the Fast Global Registration algorithm on two point clouds.

    Args:
        source_down (open3d.pcd): the source point cloud
        target_down (open3d.pcd): the target point cloud
        source_fpfh (open3d.Feature): the source FPFH features
        target_fpfh (open3d.Feature): the target FPFH features
        voxel_size (float): the voxel size used in the preparation

    Returns:
        np.array: the 4x4 global transformation matrix
    """
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source=source_down, target=target_down, 
        source_feature=source_fpfh, target_feature=target_fpfh,
        option=o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    
    return result.transformation