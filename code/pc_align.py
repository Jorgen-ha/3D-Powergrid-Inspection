"""Simple script to align the camera point cloud with the lidar point cloud.
First, a global transform is found using RANSAC, before using this transform
to perform ICP. The outputted ply files can then be merged with the original
lidar point clouds, giving a higher resolution pcd all together."""

import open3d as o3d
import numpy as np
import glob
import sys
import os

from utils import draw_registrations, find_proper_voxel_size

def pcd_prep(pcd, voxel_size, upscale=False, kd_neighbours=100):
    """Function preparing a point cloud to be used in the alignment process.

    Args:
        pcd (open3d.pcd): the point cloud to prepare
        voxel_size (float): voxel size for downsampling and feature extraction
        upscale (bool, optional): either a given scale (float) or false. Defaults to False.
        kd_neighbours (int, optional): neighbours in the feature extraction search. Defaults to 100.

    Returns:
        open3d.pcd, open3d.Feature: the downsampled point cloud and the FPFH features
    """
    pcd_down = pcd.voxel_down_sample(voxel_size) 
    
    # Upscale the point cloud if indicated
    if upscale:
        pcd_down.scale(upscale, center=pcd_down.get_center()) 
    
    # Compute normals
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=30))
    
    # Compute FPFH features
    pc_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=kd_neighbours))
    
    return pcd_down, pc_fpfh

def perform_ransac(source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
                   similarity_strictness, iterations=4000000, confidence=1.0):
    """Function performing the RANSAC global registration given two point clouds
    and their FPFH features.

    Args:
        source_down (open3d.pcd): the source point cloud
        target_down (open3d.pcd): the target point cloud
        source_fpfh (open3d.Feature): the source FPFH features
        target_fpfh (open3d.Feature): the target FPFH features
        distance_threshold (float): the maximum distance between two correspondences
        similarity_strictness (float): the strictness of the similarity check (between 0 and 1)
        iterations (int, optional): number of RANSAC iterations. Defaults to 4000000.
        confidence (float, optional): confidence score of the matched features for ransac. 
        Defaults to 1.0.

    Returns:
        np.array: the 4x4 global transformation matrix
    """
    checks = [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(similarity_strictness),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)]
        
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source=source_down, target=target_down, 
        source_feature=source_fpfh, target_feature=target_fpfh, 
        max_correspondence_distance=distance_threshold,
        mutual_filter=False,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n = 4,
        checkers = checks,
        criteria = o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=iterations, 
                                                                        confidence=confidence))

    return result.transformation

def perform_icp(source, target, init_trans, distance_threshold, iterations=30):
    """Function performing the ICP registration, locally refining the global transformation
    found earlier.

    Args:
        source (open3d.pcd): the source point cloud
        target (open3d.pcd): the target point cloud
        init_trans (np.array): the 4x4 global transformation matrix (initial alignment)
        distance_threshold (float): the distance threshold for the ICP
        iterations (int, optional): number of iterations for the ICP. Defaults to 30.

    Returns:
        np.array: the refined 4x4 transformation matrix
    """
    method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations)
    
    result = o3d.pipelines.registration.registration_icp(source=source, target=target,
                                                          max_correspondence_distance=distance_threshold,
                                                          init=init_trans,
                                                          estimation_method=method, 
                                                          criteria=criteria)
    
    return result.transformation

def visualise_correspondences(source, target, correspondences):
    """Function visualising correspondences between two point clouds, given the 
    clouds and their correspondences. 
    Credits: inspired by a similar function provided by Peter Ørnulf Ivarsen, SINTEF.

    Args:
        source (np.array): np.array of the source point cloud (xyz)
        target (np.array): np.array of the target point cloud (xyz)
        correspondences (np.array): the correspondences between the two point clouds
    """
    # Load the point clouds with open3d
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source[:,:3])

    tar_pcd = o3d.geometry.PointCloud()
    tar_pcd.points = o3d.utility.Vector3dVector(target[:,:3])
    
    # Colour the point clouds 
    src_pcd.paint_uniform_color([1, 0.706, 0])      # orange
    tar_pcd.paint_uniform_color([0, 0.651, 0.929])  # blue

    # Create a line set between the similar features in the two point clouds
    line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(
                                 src_pcd, tar_pcd, correspondences)
    line_set.paint_uniform_color([0.9, 0.6, 0.7]) # pink

    # Display the result
    print("Visualising correspondences, source in orange, target in blue, correspondences in pink")
    o3d.visualization.draw_geometries([tar_pcd, src_pcd, line_set])

def find_display_correspondences(source, target, source_fpfh, target_fpfh):
    """Function finding and displaying correspondences between two point clouds.

    Args:
        source (open3d.pcd): the source point cloud
        target (open3d.pcd): the target point cloud
        source_fpfh (open3d.Feature): the source FPFH features
        target_fpfh (open3d.Feature): the target FPFH features
    """
    # Find correspondences and visualise them
    corrs = o3d.pipelines.registration.correspondences_from_features(
                                    source_fpfh, target_fpfh, mutual_filter=False)
    # Transform the point clouds to xyz
    source.translate(target.get_center()+(0,10,0), relative=False)
    source_xyz = np.asarray(source.points)
    target_xyz = np.asarray(target.points)
    corrs = np.array(corrs)
    visualise_correspondences(source_xyz, target_xyz, corrs, save=False)

def inspection():
    """Simple function to inspect alignment during the process. No input/output but
    might generate a new file. Visualised with open3D."""
    
    name = "be_all-roi"
    abv = name[0:2]
    
    scales = {"6d": 6.789, "66": 6.623, "05": 7.130, "98": 1.253, 
              "e1": 6.692, "be": 6.728, "67": 1.265}
    UPSCALE = scales[abv]
    
    source = o3d.io.read_point_cloud(f"pcs/colmap/processed_final/{name}.ply")            
    target = o3d.io.read_point_cloud("pcs/lidar/extracted_with_ground/be_lidar.ply")
    src_voxel_size = find_proper_voxel_size(source, target_points=600, step=0.01) 
    trg_voxel_size = find_proper_voxel_size(target, target_points=600, step=0.01) 
    print(f"Estimated source voxel size: {src_voxel_size}")
    print(f"Estimated target voxel size: {trg_voxel_size}")
    source_down, source_fpfh = pcd_prep(source, src_voxel_size, upscale=UPSCALE, kd_neighbours=100)
    target_down, target_fpfh = pcd_prep(target, trg_voxel_size, kd_neighbours=100)
    print(f"Source after preparation: {source_down}")
    print(f"Target after preparation: {target_down}")    
    
    # Uncomment below two lines to visualise the found correspondences
    # source_down.translate(target_down.get_center()+(0,10,0), relative=False)
    # find_display_correspondences(source_down, target_down, source_fpfh, target_fpfh)
    
    # Specify global registration parameters
    distance_threshold = trg_voxel_size * 1.5
    similarity_strictness = 0.6
    
    global_trans = perform_ransac(source_down=source_down, target_down=target_down,
                source_fpfh=source_fpfh, target_fpfh=target_fpfh, distance_threshold=distance_threshold,
                similarity_strictness=similarity_strictness, iterations=4000000, confidence=1.0)
    print(f"Global registration transformation matrix:\n {global_trans}")
    print("Visual check of global registration....")
    draw_registrations(source_down, target_down, global_trans, recolor=True)
    more = int(input("Continue? 1 or 0: "))
    if more == 0:
        exit()  
    
    # Prepare the original point cloud for ICP
    source.scale(UPSCALE, center=source.get_center()) # makes camera pcd same scale as lidar
    distance_threshold = trg_voxel_size * 0.45
    
    # Perform ICP
    print("Performing ICP ....")
    icp_transform = perform_icp(source=source, target=target, init_trans=global_trans, 
                            distance_threshold=distance_threshold, iterations=1000)
    print(f"ICP transformation:\n {icp_transform}")
    source.transform(icp_transform)    # apply the transformation
    print("Visual check of ICP....")
    draw_registrations(source, target, recolor=True)
    more = int(input("Save? 1 or 0: "))
    if more == 0:
        exit()  
    
    # Save the aligned point cloud
    o3d.io.write_point_cloud(f"pcs/colmap/aligned_final/{name}-aligned.ply", source)
    
    # Save the found transformation
    np.savetxt(f"pcs/colmap/aligned_final/transformations/{name}-transformation.txt", icp_transform)

def align_all_pcds(lidar_dir, camera_dir, output_dir):
    """Function performing all the steps to align the given camera point clouds 
    with the given lidar point clouds. The number of point clouds must be equal
    (i.e. n_camera = n_lidar)

    Args:
        lidar_dir (str): path to the directory holding all the lidar point clouds
        camera_dir (str): path to the directory holding all the camera point clouds
        output_dir (str): output directory to store the aligned camera point clouds
    """
    lidar_paths = glob.glob(f"{lidar_dir}*.ply"); lidar_paths.sort()
    camera_paths = glob.glob(f"{camera_dir}*.ply"); camera_paths.sort()
    
    assert len(lidar_paths) == len(camera_paths), "Number of lidar and camera point clouds must be equal!"
    
    for lidar_path, camera_path in zip(lidar_paths, camera_paths):
        name = os.path.basename(camera_path).split(".")[0]
        print(f"Processing {name}....")
        abv = name[0:2]
        
        scales = {"6d": 6.789, "66": 6.623, "05": 7.130, "98": 1.253, 
                "e1": 6.692, "be": 6.728, "67": 1.265}
        UPSCALE = scales[abv]
        
        lidar_pcd = o3d.io.read_point_cloud(lidar_path)     # Read the point clouds
        camera_pcd = o3d.io.read_point_cloud(camera_path)
        
        cam_voxel_size = find_proper_voxel_size(camera_pcd, target_points=600, step=0.01) 
        lidar_voxel_size = find_proper_voxel_size(lidar_pcd, target_points=600, step=0.01)
            
        cam_pcd_down, cam_pcd_fpfh = pcd_prep(camera_pcd, cam_voxel_size, upscale=UPSCALE, kd_neighbours=100)
        lidar_pcd_down, lidar_pcd_fpfh = pcd_prep(lidar_pcd, lidar_voxel_size, kd_neighbours=100)
        
        distance_threshold = lidar_voxel_size * 1.5
        similarity_strictness = 0.6
        global_trans = perform_ransac(source_down=cam_pcd_down, target_down=lidar_pcd_down,
            source_fpfh=cam_pcd_fpfh, target_fpfh=lidar_pcd_fpfh, distance_threshold=distance_threshold,
            similarity_strictness=similarity_strictness, iterations=4000000, confidence=1.0)
        
        camera_pcd.scale(UPSCALE, center=camera_pcd.get_center()) # makes camera pcd same scale as lidar
        distance_threshold = lidar_voxel_size * 0.45
        icp_transform = perform_icp(source=camera_pcd, target=lidar_pcd, init_trans=global_trans, 
                        distance_threshold=distance_threshold, iterations=1000)
        camera_pcd.transform(icp_transform)    # apply the transformation before saving
        
        o3d.io.write_point_cloud(f"{output_dir}{name}-aligned.ply", camera_pcd)
        np.savetxt(f"{output_dir}transformations/{name}-transformation.txt", icp_transform)
    
    print(f"Finished aligning all point clouds. Output can be found in {output_dir}.")    
    

"""Input should be <lidar_mast_ground_dir>, (path to the directory holding all the lidar point clouds),
<camera_mast_ground_dir> (path to the directory holding all the camera point clouds) and <output_dir>
(the directory to store all the output, must contain trailing '/'. Created if not existing)."""
if __name__ == "__main__":
    if len(sys.argv) != 4:
        if len(sys.argv) == 2 and sys.argv[1] == "inspect":
            # If specified, inspecting all steps of the alignment process can be done
            inspection()
            exit()
        print("Usage: python3 pc-align.py [lidar_mast_ground_dir] [camera_mast_ground_dir] [output_dir]")
        sys.exit(1)
   
    lidar_dir = sys.argv[1]
    camera_dir = sys.argv[2]
    output_dir = sys.argv[3]
    if not os.path.exists(output_dir):
        os.system(f"mkdir {output_dir} {output_dir}transformations")
    if not os.path.exists(output_dir+"transformations"):
        os.system(f"mkdir {output_dir}transformations")
    
    align_all_pcds(lidar_dir, camera_dir, output_dir)
