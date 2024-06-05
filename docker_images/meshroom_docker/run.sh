# Simple script to run the meshroom container, with GUI support
podman run --rm --name meshroom --device nvidia.com/gpu=0 \
    -p 2222:22 -v /home/meshroom:/data \
    -v ~/.Xauthority:/root/.Xauthority-n:rw \
    -it meshroom:2023.3.0-av3.2.0-centos7-cuda11.3.1 \
    
