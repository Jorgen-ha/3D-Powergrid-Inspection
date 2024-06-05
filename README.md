# Exploring Photogrammetry for High-Fidelity 3D Model Generation: A Power Grid Inspection Study

This repository holds code developed for my MSc thesis project, which involved exploring different state-of-the-art methods and toolkits to reconstruct 3D models from 2D images captured by a drone in a power line inspection setting. 
Among the toolkits that were explored were Colmap, Meshroom and Nerfstudio - and their respective container images can be found in the **docker_images** directory. 
The main objective of this thesis project is to generate 3D models with higher-fidelity than those provided by an on-board LiDAR scanner on the drone performing the inspections.

The results of an implementation study will be presented first. Here, the three aforementioned frameworks were implemented, tested and compared to each other on the provided dataset. 
Lastly, the results of the final implementation will be displayed. The best performing framework from the implementation study was chosen to as the model to reconstruct 3D models moving forward, and these models were later aligned with the 3D LiDAR data. 

No example data is uploaded to this repository, as a non-disclosure agreement was signed between me and the data provider, [Stellaire](https://www.stellaire.ai/).

## Implementation study
The goal of the implementation study was to implement multiple state-of-the-art frameworks that tackle the problem of reconstructing real-world objects through images. Three frameworks were chosen, namingly [Colmap](https://colmap.github.io/), [Meshroom](https://alicevision.org/) and [Nerfstudio](https://docs.nerf.studio/). 
Colmap and Meshroom both tackle the problem using traditional photogrammetry methods, whilst Nerfstudio implements several different methods based on neural radiance fields.

### Colmap
Colmap was introduced by Schönberger et al. in 2016, and is a popular open-source software for transforming multiple unstructured 2D images into a 3D model.
Its pipeline consists of six steps: 
1. Feature extraction
2. Feature matching
3. Feature mapping
4. Image undistortion
5. Stereo patchmatching
6. Depth map fusion

All that is required as input is the images. Colmap has an automatic reconstruction pipeline that was found to perform poorly compared to doing the steps manually. The below results are therefore obtained by performing the reconstruction manually, as found in `full_reconstruction.sh` (inside `docker_images/colmap_docker`). 

The output of Colmap is displayed in the below figure, next to the equivalent LiDAR point cloud of the same mast. 
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/c4a25a6e-9a4b-4489-8afc-12e7fa1e438a)

Given that 3D models are best viewed in 3D, a video displaying the results from different angles is also provided: 

https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/9241ffec-f7c3-4890-a472-ccb22762d5bb

### Meshroom
Meshroom is a free open-source 3D reconstruction software based on the Alicevision framework, introduced by Griwodz in 2021. 
Much like Colmap it takes unstructured images of an object or scene as input, and outputs a 3D model after performing its photogrammetry pipeline. 
Meshroom differs from Colmap in several aspects when it comes to how features are extracted and matched, how depth maps are created, and also how the final model is fused together. To execute the pipeline, the path to the directory holding the images should be inputted, along with a desired output destination (as seen in `cmd_line_meshroom.sh`, inside `docker_images/meshroom_docker`). 

The output of Meshroom is displayed in the below figure, next to the equivalent LiDAR point cloud of the same mast.
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/5dfcb11b-749b-4147-93d5-4fb00ef34e9d)

A video displaying the reconstruction from more angles can be found below:

https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/7170eea0-4636-4825-98ac-82f9b93667f7

### Nerfstudio
Nerfstudio is an open-source initiative focused on being a contributor-friendly and easy-to-work-with API for exploring Neural Radiance Fields (NeRFs). Instead of relying on photogrammetry methods, NeRFs attempt to render views that are not captured in the images directly, but can be inferred from them using traditional rendering techniques.
To be able to infer novel views from the provided images, the camera positions must be known. This is not the case in the provided data, so a necessary preprocessing step is carried out first. Here, Colmap is used to estimate the camera poses. All the steps can be found in `custom_run,sh` (inside `docker_images/nerfstudio_docker`). 

For this implementation, the Instant-NGP model provided by Nerfstudio was the preferred network, and its results are displayed in the below figure.
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/21ce8c4f-903f-410a-87da-ed3e535d8d01)

A video can also be found below:

https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/04a4918f-c4f5-4772-bd71-e03cf25581f2

### All frameworks vs LiDAR
To inspect the results of all three frameworks compared to the LiDAR point cloud, the below video was made. Here, the LiDAR point cloud is displayed at the bottom, with the Colmap, Meshroom and NeRF results above (in that order, from left to right). 

https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/e860c2af-46f3-483a-b1cd-e65fd4e4bd21

From the above results, it is clear that Colmap provided the overall most accurate reconstruction. This was therefore the method chosen for the final implementation.

## Final implementation
The final implementation was concerned with generating the best possible reconstruction from the images, before aligning this model with the LiDAR point cloud. This way, not only would a model preserving fine-grained details be obtained, but it would also be geometrically correct and positioned/oriented in the exact same way the real-world mast is.

The pipeline of the final implementation is as follows: 

0. Capture the data (both images and LiDAR point clouds) during an inspection run
1. Input the images and LiDAR clouds to each their own preprocessing pipe
2. Process the images to obtain point clouds that can be aligned later (`pre_process_camera.py`)
	1. Input all images to Colmap to obtain a dense 3D reconstruction
	2. Use Open3D to identify the mast in the reconstructed scene
	3. Crop the Colmap-generated point cloud to hold only the region of interest around the mast
3. Process the LiDAR point clouds (`pre_process_lidar.py`)
	1. Find and extract the masts that have been photographed from the LiDAR point clouds
	2. Crop the point cloud holding all the masts into several smaller point clouds holding the areas of interest for each mast that is also photographed
4. Align the two point cloud sources (`pc_align.py`)

### Results
To assess the performance of the proposed implementation, qualitative and quantitative metrics have been derived. Below, a few of these are displayed. Both the density of the final point clouds and the precision of the alignment will be evaluated. 
The two main objectives were: 
1. Obtain point clouds having 10 times the amount of points the LiDAR point clouds hold
2. Align the dense point clouds within a distance resulting in a root mean squared error of less than 14 cm, when compared to the original position of the mast in the LiDAR point cloud. 

#### Point Cloud Density
First is a visual comparison of one of the masts in both the form of a LiDAR point cloud and a Colmap point cloud. The density of the Colmap point cloud can be seen to be far beyond that of the LiDAR. 
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/b47d6ddb-107f-4072-968e-6f14b32ed12f)

The notion of the Colmap point cloud appearing denser is backed by the following table, in which three density metrics are calculated for all the point clouds after all but the mast points have been removed. 
These metrics are all based on a sphere which radius is the diameter of a standard insulator disc (28 cm). These three are; 
- **Mean NN**, mean nearest neighbours, denoting how many neighbouring points are within the specified sphere
- **Surface density**, denoting the number of neighbouring points divided by the neighbourhood surface area of the sphere 
- **Volume density**, denoting number of neighbours divided by the neighbourhood volume of the sphere
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/ac09d198-487a-43e6-bdbb-a09c28332e4b)

From the table, an average 55 times increase in point density can be found for the Colmap point clouds compared to the baseline LiDAR point cloud.

#### Alignment Accuracy
Aligning the much denser Colmap point clouds with the LiDAR equivalent was done using global registration and the ICP algorithm on the point clouds holding only the region of interest. 
One such result is displayed below, with the Colmap point cloud in red and the LiDAR point cloud in yellow. 
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/9289f659-a181-4ff1-8943-89b9d50f0906)

To determine the accuracy of the alignment for all seven masts, the metrics in the following table were derived based on the absolute point-cloud-to-point-cloud distances after alignment:
- **RMSE**, the root mean squared error of the absolute distances in the point cloud
- **Mean**, the mean absolute distance of the point cloud
- **STD**, the standard deviation from the mean 
![image](https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/d76af70d-bea9-4838-98d7-da25cfebf8d7)

From the above table it can be seen that five out of seven masts achieved an alignment within the specified objective of an RMSE less than 0.14 m (14 cm). 

#### Final output
Finally, to display the potential of this implementation in a 3D inspection setting, the aligned Colmap point clouds holding denser representations of the masts are put inside the LiDAR point cloud. 
The result is the full scene that has been inspected using the LiDAR sensor, combined with much denser masts that are preserving even fine-grained details of the components. 

A short video displaying this scene can be found below, side-by-side with the default quality of the captured LiDAR point cloud (left).

https://github.com/Jorgen-ha/3D-Powergrid-Inspection/assets/51888629/b9c98397-560c-42bf-8e77-9e93f1c8331d

### Conclusion
This thesis project explored ways of obtaining dense 3D models from high-resolution images, and how to align these point clouds to much sparser but correctly scaled point clouds. 
An implementation requiring no human input, just the collected image and LiDAR data, has been proposed, and its performance on the provided dataset evaluated. 
The implementation succeeds in creating point clouds that hold a lot more detail than the ones being captured at the moment, which is evident from all of the objectives being achieved. 
