# Depth Estimation and 3D Point Cloud Generation
## Objective: 
Implement a ROS2 node that performs depth estimation from RGB-D or monocular camera inputs, generates 3D point clouds, and performs segmentation and analysis for downstream  perception tasks.

## Deliverables:
1. ROS2 node with stereo (or monocular) depth estimation, point cloud generation and filtering
nodes
2. Quantitative error analysis report with plots (optional)
3. Demo video of RViz2 visualization of estimated depth and segmented point cloud
4. Dockerfile or docker image with all dependencies (Optional)
5. GitHub repository with README.md --> **readme_exercise.md**

## Submission:
1. Create a comprehensive README.md in your GitHub repository that clearly explains your project.
It should include:
    - Step-by-step instructions for setting up and running the project
    - All deliverables, such as visualizations, embedded directly within the README
    - A link to your GitHub repository
    - The path or instructions to access the containerized Docker image of your work (either the Dockerfile in your repo or the Docker Hub link)


2. Ensure that your README is detailed enough for someone unfamiliar with your project to follow and reproduce your results.
3. Once complete, upload your GitHub repository via ILIAS for submission.

## Step-by-step instructions
Using WSL on Windows

Fork the repo, then execute the below, shown the consoe output with failure:

```bash
chrisk@iemob-nb50:~/workspace/px4-sim$ ./build.sh --base
[INFO] Building image (Ubuntu 24.04 LTS + ROS 2 Jazzy + Gazebo Harmonic)
[+] Building 652.1s (8/16)                                                                               docker:default
 => [internal] load build definition from ros2-jazzy-gazebo-harmonic.Dockerfile                                    0.0s
 => => transferring dockerfile: 2.58kB                                                                             0.0s
 => [internal] load metadata for docker.io/library/ubuntu:24.04                                                    1.0s
 => [auth] library/ubuntu:pull token for registry-1.docker.io                                                      0.0s
 => [internal] load .dockerignore                                                                                  0.0s
 => => transferring context: 2B                                                                                    0.0s
 => [internal] load build context                                                                                  0.0s
 => => transferring context: 38B                                                                                   0.0s
 => CACHED [ 1/11] FROM docker.io/library/ubuntu:24.04@sha256:c4a8d5503dfb2a3eb8ab5f807da5bc69a85730fb49b5cfca233  0.0s
 => => resolve docker.io/library/ubuntu:24.04@sha256:c4a8d5503dfb2a3eb8ab5f807da5bc69a85730fb49b5cfca2330194ebcc4  0.0s
 => [ 2/11] RUN apt-get update && apt-get install -y     locales     && locale-gen en_US en_US.UTF-8     && upd  322.8s
 => ERROR [ 3/11] RUN apt-get update && apt-get install -y     git     curl     lsb-release     gnupg2     soft  328.2s
------
 > [ 3/11] RUN apt-get update && apt-get install -y     git     curl     lsb-release     gnupg2     software-properties-common     && rm -rf /var/lib/apt/lists/*:
11.96 Get:1 http://security.ubuntu.com/ubuntu noble-security InRelease [126 kB]
18.48 Err:2 http://archive.ubuntu.com/ubuntu noble InRelease
18.48   400  Bad Request [IP: 91.189.92.24 80]
39.51 Get:3 http://archive.ubuntu.com/ubuntu noble-updates InRelease [126 kB]
41.58 Get:4 http://security.ubuntu.com/ubuntu noble-security/main amd64 Packages [2025 kB]
53.33 Get:5 http://archive.ubuntu.com/ubuntu noble-backports InRelease [126 kB]
57.18 Get:6 http://archive.ubuntu.com/ubuntu noble-updates/main amd64 Packages [2402 kB]
72.66 Get:7 http://security.ubuntu.com/ubuntu noble-security/restricted amd64 Packages [3581 kB]
83.10 Get:8 http://security.ubuntu.com/ubuntu noble-security/multiverse amd64 Packages [34.8 kB]
121.9 Get:9 http://archive.ubuntu.com/ubuntu noble-updates/multiverse amd64 Packages [38.5 kB]
148.8 Ign:10 http://security.ubuntu.com/ubuntu noble-security/universe amd64 Packages
154.1 Get:10 http://security.ubuntu.com/ubuntu noble-security/universe amd64 Packages [1507 kB]
158.8 Get:11 http://archive.ubuntu.com/ubuntu noble-updates/restricted amd64 Packages [3752 kB]
227.6 Get:12 http://archive.ubuntu.com/ubuntu noble-updates/universe amd64 Packages [2156 kB]
262.8 Get:13 http://archive.ubuntu.com/ubuntu noble-backports/universe amd64 Packages [36.1 kB]
288.9 Ign:14 http://archive.ubuntu.com/ubuntu noble-backports/multiverse amd64 Packages
305.8 Get:15 http://archive.ubuntu.com/ubuntu noble-backports/main amd64 Packages [49.5 kB]
327.9 Get:14 http://archive.ubuntu.com/ubuntu noble-backports/multiverse amd64 Packages [695 B]
327.9 Reading package lists...
328.2 E: Failed to fetch http://archive.ubuntu.com/ubuntu/dists/noble/InRelease  400  Bad Request [IP: 91.189.92.24 80]
328.2 E: The repository 'http://archive.ubuntu.com/ubuntu noble InRelease' is not signed.
------
ros2-jazzy-gazebo-harmonic.Dockerfile:22
--------------------
  21 |     # Install essential tools
  22 | >>> RUN apt-get update && apt-get install -y \
  23 | >>>     git \
  24 | >>>     curl \
  25 | >>>     lsb-release \
  26 | >>>     gnupg2 \
  27 | >>>     software-properties-common \
  28 | >>>     && rm -rf /var/lib/apt/lists/*
  29 |
--------------------
ERROR: failed to build: failed to solve: process "/bin/sh -c apt-get update && apt-get install -y     git     curl     lsb-release     gnupg2     software-properties-common     && rm -rf /var/lib/apt/lists/*" did not complete successfully: exit code: 100
chrisk@iemob-nb50:~/workspace/px4-sim$
```

