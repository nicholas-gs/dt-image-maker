# rpi/bullseye

> **tl;dr;** Using a fresh debian bullseye raspberry pi os image, install the necessary programs/dependencies.

## Setup

1. Raspberry Pi 3B+
2. raspberry pi OS (64 bit) Debian Bullseye

## Camera/Opencv/gstreamer

1. add `dtoverlay=imx219` to `/boot/config.txt`

2. Make sure the camera is working by taking a photo
```bash
libcamera-jpeg -o test.jpg
```
3. Install opencv with gstreamer support following [this](https://lindevs.com/install-precompiled-opencv-on-raspberry-pi) guide.

```bash
wget https://github.com/prepkg/opencv-raspberrypi/releases/latest/download/opencv_64.deb

sudo apt install -y ./opencv_64.deb
```

4. You can also run the following command which opens a video feed in a new window
```bash
gst-launch-1.0 libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! videoscale ! clockoverlay time-format="%D %H:%M:%S" ! autovideosink
```

5. Run the python script to make sure that opencv and gstreamer works.
```bash
# Takes a photo and saves it in the same directory
python3 opencv_gstreamer_photo.py
```

## Docker
```bash
# Use a shell script provided by docker
curl -fsSL https://get.docker.com -o get-docker.sh

chmod +x get-docker.sh

./get-docker.sh

sudo groupadd docker && sudo usermod -aG docker $USER

# Make sure everything works
docker run hello-world

# Example ROS2 publisher
docker run -it --net=host --ipc=host --entrypoint /bin/bash --rm nicholasgs/dt-cps:rpi-foxy-test
# In the docker container
ros2 run minimal_publisher talker
```


