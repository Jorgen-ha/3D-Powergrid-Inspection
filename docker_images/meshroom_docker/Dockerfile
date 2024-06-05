# Credits: https://github.com/alicevision/Meshroom/tree/develop & 
# https://github.com/colmap/colmap/blob/main/docker/Dockerfile

ARG UBUNTU_VERSION=20.04
ARG NVIDIA_CUDA_VERSION=12.3.2

# Docker builder stage.
FROM nvcr.io/nvidia/cuda:${NVIDIA_CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION} as builder

# Prevent stop building ubuntu at time zone selection.
ENV DEBIAN_FRONTEND=noninteractive

# Prepare and empty machine for building.
RUN apt-get update && \
    apt-get install -y --no-install-recommends --no-install-suggests \
    wget

# Get the meshroom binary files and extract them to new folder 'meshroom'.
RUN wget -N https://github.com/alicevision/meshroom/releases/download/v2023.3.0/Meshroom-2023.3.0-linux.tar.gz && \
    tar -xvf Meshroom-2023.3.0-linux.tar.gz --no-same-owner && \
    rm Meshroom-2023.3.0-linux.tar.gz

# Docker runtime stage.
FROM nvcr.io/nvidia/cuda:${NVIDIA_CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION} as runtime

# Install necessary runtime dependencies.
RUN apt-get update && \
    apt-get install -y --no-install-recommends --no-install-suggests \
    libkrb5-dev

# Copy the Meshroom binaries from the builder stage.
COPY --from=builder /Meshroom-2023.3.0 /usr/local/
