import matplotlib.cm as cm
import open3d as o3d
import numpy as np
import glob
import copy
import sys
import os

from utils import find_proper_voxel_size, find_pcd_height

def filter_outliers(pcd, nb_points=2, radius=0.2, visualise=False):
    """Function performing radius outlier removal on a point cloud using open3D.

    Args:
        pcd (open3d.pcd): the point cloud to remove outliers from
        nb_points (int, optional): minimum number of neighbours to be an inlier. Defaults to 2.
        radius (float, optional): max distance between neighbours. Defaults to 0.2.
        visualise (bool, optional): whether or not to visualise. Defaults to False.

    Returns:
        open3d.pcd: the inlier point cloud
    """
    inlier_cloud, inlier_idx = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    
    if visualise:
        inlier_points = pcd.select_by_index(inlier_idx)
        inlier_points.paint_uniform_color([1, 0, 0]) # inliers are red
        outlier_points = pcd.select_by_index(inlier_idx, invert=True)
        outlier_points.paint_uniform_color([0, 0, 1]) # outliers are blue
        print("Visualising the point cloud with outliers removed. Inliers are red, outliers blue.")
        print(f"Used radus outlier removal, with radius: {radius}, nb_points: {nb_points}")
        o3d.visualization.draw_geometries([inlier_points, outlier_points]) 
        
    return inlier_cloud

def perform_plane_segmentation(pcd, distance_threshold=0.1, ransac_n=3, num_iterations=1000, visualise=False):
    """Function performing plane segmentation on a point cloud using open3D.

    Args:
        pcd (open3d.pcd): the point cloud to segment
        distance_threshold (float, optional): maximum distance a point can have to an estimated 
        plane to be considered an inlier. Defaults to 0.1.
        ransac_n (int, optional): number of points used to estimate a plane. Defaults to 3.
        num_iterations (int, optional): how often a random plane is sampled and verified. 
        Defaults to 1000.
        visualise (bool, optional): whether or not to visualise the results. Defaults to False.
    
    Return:
        np.array: the plane model
        np.array: the inlier point cloud
        np.array: the outlier point cloud
    """
    tmp_pcd = copy.deepcopy(pcd)
    plane_model, inliers = tmp_pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    
    inlier_cloud = tmp_pcd.select_by_index(inliers)
    outlier_cloud = tmp_pcd.select_by_index(inliers, invert=True)
    if visualise:
        print("Visualising the segmented plane (ground) of the point cloud")
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        print("Points belonging to the plane are red")
        if len(np.asarray(outlier_cloud.points)) == 0:
            print("Found all points to be along the plane")
        vis_cloud = copy.deepcopy(inlier_cloud)
        vis_cloud.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([vis_cloud, outlier_cloud])
        
    return plane_model, inlier_cloud, outlier_cloud

def perform_dbscan(pcd, eps=2.5, min_points=10, print_progress=False, visualise=False):
    """Function performing DBSCAN clustering on a point cloud, as per open3D's implementation.

    Args:
        pcd (open3d.pcd): the point cloud to cluster
        eps (float, optional): distance to neighbours in a cluster. Defaults to 2.5.
        min_points (int, optional): minimum number of points to form a cluster. Defaults to 10.
        print_progress (bool, optional): whether or not to print the progress. Defaults to False.
        visualise (bool, optional): whether or not to visualise the results. Defaults to False.

    Returns:
        np.array: the labels of the clusters
    """
    labels = np.array(pcd.cluster_dbscan(eps, min_points, print_progress))

    if visualise:
        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = cm.get_cmap('tab20')(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        print("Visualising the clustered point cloud after DBSCAN")
        o3d.visualization.draw_geometries([pcd])
    
    return labels

def find_mast_bbox(pcd, plane_model, voxel_size, visualise=False):
    """Function calculating the bounding box of the mast in the point cloud.

    Args:
        pcd (open3d.pcd): the point cloud containing the mast
        plane_model (np.array): model of the plane found by plane segmenting the point cloud
        voxel_size (float): the voxel size used when the point cloud was downsampled
        visualise (bool, optional): whether or not to visualise the results. Defaults to False.

    Returns:
        open3d.oriented.bounding.box (upon success): the bounding box encapulating the mast
        None (upon failure): if no vertical points were found (i.e. no mast)
    """
    tmp_pcd = copy.deepcopy(pcd)
    
    [a,b,c,d] = plane_model
    plane_normal = np.array([a, b, c])
    
    # Find the vertical points of the pointcloud
    vertical_pcd = find_perpendicular_points(tmp_pcd, plane_normal, visualise=visualise)
    
    if vertical_pcd is None:
        return 
    
    # Filter the vertical points to only hold the mast
    org_center = tmp_pcd.get_center()
    mast_pcd = find_mast_points(vertical_pcd, voxel_size, org_center, visualise=visualise)

    # Find the mean distance of all the mast points from the center
    mean_distance = np.mean(np.linalg.norm(np.asarray(mast_pcd.points) - mast_pcd.get_center(), axis=1))
    
    # Make a cylinder around the center big enough to capture the mast
    r = mean_distance+mean_distance/2
    h = find_pcd_height(vertical_pcd)
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=h)
    
    # Rotate the cylinder to be perpendicular to the previously found plane
    R = calculate_rotation_matrix(plane_normal)
    cylinder.rotate(R, center=tmp_pcd.get_center())
    cylinder.translate(mast_pcd.get_center(), relative=False) # translate to the center of the mast
    bbox_cylinder_rot = cylinder.get_oriented_bounding_box()
    
    if visualise:
        bbox_cylinder_rot.color = [0, 1, 0]
        print("Visualising the found bounding box of the mast.")
        o3d.visualization.draw_geometries([tmp_pcd, bbox_cylinder_rot])
    
    return bbox_cylinder_rot

def filter_wires(pcd, voxel_size=0.55, visualise=False):
    """Function filtering out the wires from the cropped point cloud. 
    It first finds the ground plane, before finding the mast plane, removing all points
    that are not part of either.

    Args:
        pcd (open3d.pcd): the point cloud to filter
        voxel_size (float, optional): voxel size used for downsampling. Defaults to 0.55.
        visualise (bool, optional): whether or not to visualise. Defaults to False.

    Returns:
        open3d.pcd: point cloud with the wires removed
    """
    _, ground_pcd, all_but_plane_pcd = perform_plane_segmentation(pcd, distance_threshold=voxel_size-0.05, visualise=visualise)
    _, mast_pcd, _ = perform_plane_segmentation(all_but_plane_pcd, distance_threshold=voxel_size-0.05, visualise=visualise)

    
    # Merge the mast_pcd and the ground_pcd
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd += ground_pcd
    merged_pcd += mast_pcd
    
    if visualise:
        print("Visualising the point cloud who's wires shall be removed")
        o3d.visualization.draw_geometries([pcd])
        print("Visualising the point cloud without the wires")
        final_pcd = copy.deepcopy(merged_pcd)
        o3d.visualization.draw_geometries([final_pcd]) 
    
    return merged_pcd

def find_perpendicular_points(pcd, plane_normal, visualise=False):
    """Function finding all vertical points with respect to the plane normal.

    Args:
        pcd (open3d.pcd): the point cloud the plane normal was calculated on
        plane_normal (np.array): the plane normal 
        visualise (bool, optional): whether or not to visualise the result. Defaults to False.

    Returns:
        open3d.pcd (upon success): the point cloud holding only the vertical points 
        None (upon failure): if no vertical points were found
    """
    
    
    tmp = copy.deepcopy(pcd)
    
    # Compute the normals of the point cloud
    tmp.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2.5, max_nn=100))
    
    vertical_points = []
    for i, normal in enumerate(np.asarray(tmp.normals)):
        angle = np.arccos(np.dot(normal, plane_normal))
        if angle > np.pi/2 - np.pi/4 and angle < np.pi/2 + np.pi/4:
            vertical_points.append(np.asarray(tmp.points)[i])
    
    if len(vertical_points) == 0:
        return
    
    # Generate a new point cloud of the vertical points
    vertical_pcd = o3d.geometry.PointCloud()
    vertical_pcd.points = o3d.utility.Vector3dVector(vertical_points)
    
    if visualise:
        vis_pcd = copy.deepcopy(vertical_pcd)
        vis_pcd.paint_uniform_color([1, 0, 0]) # the vertical points are red 
        print(f"Visualising the point cloud with its vertical points in red.")
        o3d.visualization.draw_geometries([tmp, vis_pcd])
    
    return vertical_pcd

def find_mast_points(pcd, voxel_size, org_center, visualise=False):
    """Function finding the mast points using DBSCAN clustering on the vertical points.
    The mast points are assumed to be the cluster closest to the center of the vertical point cloud.

    Args:
        pcd (open3d.pcd): the (vertical) point cloud to find the mast points in
        voxel_size (float): voxel size used to downsample the point cloud
        org_center (np.array): the center of the original point cloud
        visualise (bool, optional): whether or not to visualise the results. Defaults to False.

    Returns:
        open3d.pcd: the point cloud holding the mast points
    """
    tmp_pcd = copy.deepcopy(pcd)
    
    # Perform DBSCAN clustering on the vertical points
    labels = perform_dbscan(tmp_pcd, eps=voxel_size*2.5, min_points=15, print_progress=False, visualise=visualise)
    
    # Find the cluster closes to the center of the original point cloud
    min_distance = np.inf
    mast_label = None
    mast_pcd = o3d.geometry.PointCloud()
    for _, label in enumerate(labels):
        cluster = np.asarray(tmp_pcd.points)[labels == label]
        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(cluster)
        cluster_center = cluster_pcd.get_center()
        distance = np.linalg.norm(org_center - cluster_center)
        if distance < min_distance and label != -1:
            min_distance = distance
            mast_label = label
            mast_pcd = cluster_pcd
    
    if visualise:
        vis_pcd = copy.deepcopy(mast_pcd)
        vis_pcd.paint_uniform_color([0, 1, 0])
        center_pcd = o3d.geometry.PointCloud()
        center_pcd.points = o3d.utility.Vector3dVector([org_center])
        center_pcd.paint_uniform_color([1, 0, 0])
        print("Visualising the mast points, extracted after DBSCAN of vertical points.")
        o3d.visualization.draw_geometries([tmp_pcd, vis_pcd, center_pcd])
    
    return mast_pcd

def calculate_rotation_matrix(plane_normal):
    """Function calculating a rotation matrix that is parallel to the plane normal.
    (As per Rodrigues rotation formula: https://mathworld.wolfram.com/RodriguesRotationFormula.html)

    Args:
        plane_normal (np.array): the normal of the plane

    Returns:
        np.array: the rotation matrix
    """
    z_axis = np.array([0, 0, 1])
    rot_axis = np.cross(z_axis, plane_normal)
    
    theta = np.arccos(np.dot(z_axis, plane_normal))
    theta = np.clip(theta, -np.pi, np.pi) 
    
    rot_axis_hat = np.array([[0,            -rot_axis[2],  rot_axis[1]],
                             [rot_axis[2],     0,         -rot_axis[0]],
                             [-rot_axis[1], rot_axis[0],       0]])
    
    R = np.eye(3) + (1-np.cos(theta))*np.dot(rot_axis_hat, rot_axis_hat) + np.sin(theta)*rot_axis_hat
    
    return R


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 pre-process-camera.py [camera_pcd_dir] [output_dir] [visualise (1 or 0)]")
        sys.exit(1)
    cam_dir_path = sys.argv[1]
    output_dir = sys.argv[2]
    visualise = bool(int(sys.argv[3]))
    if not os.path.exists(output_dir):
        os.system(f"mkdir {output_dir}")

    all_pcd_paths = glob.glob(cam_dir_path + "*.ply")
    for pcd_name in all_pcd_paths:
        pcd = o3d.io.read_point_cloud(pcd_name)
        voxel_size = find_proper_voxel_size(pcd, target_points=3300, step=0.05)
        print(f"Found the voxel size to be: {voxel_size}")
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_filtered = filter_outliers(pcd_down, nb_points=2, radius=voxel_size+0.05, visualise=visualise)
        plane_model, _, _ = perform_plane_segmentation(pcd_filtered, distance_threshold=voxel_size-0.03, ransac_n=3, num_iterations=2000, visualise=visualise)
        mast_bbox = find_mast_bbox(pcd_filtered, plane_model, voxel_size, visualise=visualise)
        if mast_bbox is None:
            print("ERROR: No mast found")
            continue
        roi = pcd.crop(mast_bbox)
        roi = filter_wires(roi, voxel_size=voxel_size, visualise=visualise)
        basename = os.path.basename(pcd_name)
        name = os.path.splitext(basename)[0]
        o3d.io.write_point_cloud(f"{output_dir}{name}-roi.ply", roi)
        
    print(f"Finished processing the camera point clouds, they are found here: {output_dir}")
