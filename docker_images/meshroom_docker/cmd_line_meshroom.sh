# Simple script to perform the basic Meshroom 3D reconstruction from the 
# command line only

# Ask user for the path to the image folder
echo "Enter the path to the image folder."
read -p "> " image_folder_path

# Ask user for the path to the output folder
echo "Enter the path to the output folder."
read -p "> " output_folder_path

# Run it 
./meshroom/Meshroom-2023.3.0/meshroom_batch \
    -i $image_folder_path \
    -o $output_folder_path 

