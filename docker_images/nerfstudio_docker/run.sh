# Give the container access to nvidia GPU (required).
# -u $(id -u) \ To prevent abusing of root privilege, please use custom user privilege to start.
# Mount a folder from the local machine into the container to be able to process them (required).
# Mount cache folder to avoid re-downloading of models everytime (recommended).
# Map port from local machine to docker container (required to access the web interface/UI).
# Remove container after it is closed (recommended).
# Start container in interactive mode.
# Increase memory assigned to container to avoid memory limitations, default is 64 MB (recommended).
# Docker image name if you pulled from docker hub.
podman run --device nvidia.com/gpu=0 \
            -v /home/nerfstudio:/workspace/ \
            -v /home/$USER/.cache/:/home/user/.cache/ \
            -p 7007:7007 \
            --rm \
            -it \
            --shm-size=12g \
            nerfstudio
