## Feed-the-MiRo Demo and Setup

This README provides instructions on how to run the Feed-the-MiRo demo and outlines the necessary modifications when using the base `miro-docker` image.

-----

## Demo Instructions: How to Run Feed-the-MiRo

Follow these steps to set up and run the demo using the `miro-docker` environment.
 Clone this Repository:

    ```bash
    git clone https://github.com/acg22sb/COM3528-pre-.git
    ```

### MiRo-Docker Setup

1.  Navigate and Start the Container:
    Navigate into the provided `miro-docker` repository directory and execute the startup script:

    ```bash
    cd miro-docker
    ./miro-docker.sh start
    ```

    > Wait for the MiRo Docker image and container to be built. Press Enter if prompted to provide names.

2.  Launch the MiRo Hub Terminal:
    Access the terminal in the running MiRo-Docker container:

    ```bash
    miro-hub term
    ```

3.  Find MiRo Workspace Path:
    Within the `miro-hub term`, navigate to the ROS workspace source directory and find its path:

    ```bash
    cd mdk/catkin_ws/src
    pwd
    # Store this path for the next step (e.g., /home/miro/mdk/catkin_ws/src)
    ```

### Copying the `feed_the_miro_pkg`

1.  Find Local Package Path:
    Open a new terminal window. Navigate into your this cloned repository and find `feed_the_miro_pkg` and its absolute path:

    ```bash
    cd /path/to/your/local/feed_the_miro_pkg  # Replace with actual path
    pwd
    # Store this path (e.g., /home/user/projects/feed_the_miro_pkg)
    ```

2.  Copy the Package into Docker:
    Use the `docker cp` command with the paths found in the previous steps:

    ```bash
    docker cp <output of second pwd> miro-docker:<output of first pwd>
    ```

    Example:
    `docker cp /home/user/my_projects/feed_the_miro_pkg miro-docker:/home/miro/mdk/catkin_ws/src`

3.  Verify Copy:
    Return to the `miro-hub term` and check that the package `feed_the_miro_pkg` has been successfully copied into `mdk/catkin_ws/src`.

### MiRo Robot Connection and Configuration

1.  Connect to Dia Lab Wi-Fi (or a network where MiRo is accessible).

2.  Set MiRo Mode and Configure IP:
    Use the following commands in the `miro-hub term` to connect and configure network settings:

    ```bash
    miro mode robot
    miro ip update # Enter the MiRo's IP address
    export MIRO_LOCAL_IP=<laptop ip> # Replace with your host machine's IP
    source ~/.bashrc
    ```

3.  Check Connection:
    Verify the connection by checking if the MiRo GUI is functional:

    ```bash
    miro gui
    ```

### Make and Run YOLO Container
  Build and run the separate YOLO container and launch the server

  1. Navigate into COM3528-pre-
     
    ```bash
    cd COM3528-pre-
    ```

  2. Build the Docker Container
     
    ```bash
    ./yolo-docker.sh build
    ```
    
  3. Start and enter the Docker Container
     
    ```bash
    ./yolo-docker.sh start
    ./yolo-docker.sh term
    ```

  4. Start the Flask Server containing the YOLO detection model, ready to communicate with miro-docker
     
    ```bash
    python3 yolo_server.py
    ```

### Building and Launching the Demo

1.  Build the Package:
    Ensure you are in the `mdk/catkin_ws` directory within the `miro-hub term`:

    ```bash
    cd mdk/catkin_ws
    catkin build
    ```

2.  Source the Setup:

    ```bash
    source devel/setup.sh
    source ~/.bashrc
    ```

3.  Verify Package:

    ```bash
    rospack find feed_the_miro_pkg
    ```

4.  Launch the Demo:
    Start the main ROS launch file to run Feed-the-MiRo:

    ```bash
    roslaunch feed_the_miro_pkg feed_the_miro.launch
    ```

-----

## Overview of Changes from `miro-docker`

The Feed-the-MiRo demo is based on the provided `miro-docker` container with the following key additions/changes:

  * New ROS Package Integration:
      * The `feed_the_miro_pkg` is introduced. This package contains the core logic for the demo.
      * The demo relies on an external YOLO detection container for object recognition. The base `miro-docker` image does not include this YOLO model The launch file assumes this YOLO node is running and publishing detection results on a specific port.
  * Launch File:
      * The `feed_the_miro.launch` file is created for the initialisation of all nodes. 
