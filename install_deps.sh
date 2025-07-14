# utilities
sudo apt install -y unzip rsync ros-dev-tools librange-v3-dev 

# opentera-webrtc-ros packages
sudo apt install -y ros-humble-camera-info-manager ros-humble-rtabmap-ros ros-humble-rqt-tf-tree ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description ros-humble-turtlebot3-navigation2 ros-humble-joint-state-publisher-gui ros-humble-moveit-servo ros-humble-moveit-ros-planning-interface ros-humble-rclpy-message-converter

# protobuf
sudo apt install -y libprotobuf-dev protobuf-compiler python3-protobuf

# python dependencies
sudo apt install -y python3-pip portaudio19-dev

# nodejs dependencies
sudo apt install -y nodejs npm

# audio_utils packages
sudo apt install -y cmake build-essential gfortran texinfo libasound2-dev libpulse-dev 'libgfortran-*-dev'

# odas_ros packages
sudo apt install -y libfftw3-dev libconfig-dev

# gstreamer for hardware acceleration
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools

# qt submodules
sudo apt install -y libqt5charts5-dev

sudo apt install -y python-sphinx-build

sudo apt install -y ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3 python3-sphinx libglib2.0-dev libgtk-3-dev libpulse-dev libasound2-dev tree

sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools libunwind-dev

sudo apt-get install -y librange-v3-dev

sudo apt install -y omniorb omniorb-nameserver libomniorb4-dev omniidl omniorb-idl

# clone this repo with --recursive submodules

sudo python3 -m install omniorb-py

sudo python3 -m pip install torchvision

cd opentera-webrtc-ros/opentera_client_ros
sudo python3 -m pip install -r requirements.txt

cd ..

cd opentera_webrtc_ros
sudo python3 -m pip install -r requirements.txt

cd opentera-webrtc
sudo python3 -m pip install -r requirements.txt

cd ../../..
