# ros2-jetson-nano-image-maker

> **tl;dr;** Build sd-card flashable ROS2 and Duckietown images for [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) dev kits using Docker. Extended from this great [repository](https://github.com/defunctzombie/jetson-nano-image-maker).

## Images

1. Base Image - Prepare a base image with necessary files and setup.
2. ROS2 Foxy - Install ROS2 Foxy on top of base image
3. Duckietown - Compile [Duckietown codebase](https://github.com/nicholas-gs/duckietown_ros2_cps) on top of ROS2 Foxy

Each image builds on top of the previous. Some can be found [here](https://hub.docker.com/r/nicholasgs/dt-cps) on Dockerhub.

## Credentials

The default credentials:

username: `jetson`  
password: `jetson`

## Local Development

One advantage of using Docker to setup the root file system is the ability to iterate locally and test your changes.

Here are a few commands you can use to work locally and make sure everything installs before you push your changes to CI.

### Set Environment Variables
```
export JETSON_ROOTFS_DIR=/tmp/jetson-builder/rootfs
export JETSON_BUILD_DIR=/tmp/jetson-builder/build
export JETSON_NANO_BOARD=jetson-nano
```

### Build the rootfs image

Go to the dockerfile you want to build.

```
docker buildx build --platform linux/arm64 -t <image-name> .
```

### Run the image (without any init system)

```
docker run -it --rm --user 1000:1000 <image-name> /bin/bash
```

### Run the built image and invoke systemd init to see what runs on startup

```
docker run -it --rm --cap-add SYS_ADMIN -v /sys/fs/cgroup/:/sys/fs/cgroup:ro <image-name> /sbin/init
```

### Make a flashable image

If you are on linux, you can turn the Docker image into a flashable image

```shell
# Export the rootfs image to a folder on your file-system
# Nvidia l4t tools turn this folder into a .img file you can flash
docker export $(docker create --name nano-rootfs --platform linux/arm64 <image-name>) -o rootfs.tar

mkdir -p /tmp/jetson-builder/rootfs
sudo tar --same-owner -xf rootfs.tar -C /tmp/jetson-builder/rootfs

# Create a jetson.img from the `rootfs` you can flash to an SD card
sudo -E ./create-image.sh
```

## Post Installation

After booting into the OS on your Jetson Nano, you might want to do the following steps:

### Change environment variables

Go to `/home/jetson/duckietown/config` and change the values in the `robot_*` files. Source `/home/jetson/.bashrc` to take effect. Necessary for Duckietown codebase.

### Connect to wifi

Connect to wifi using

`sudo nmcli dev wifi connect "<network-ssid>" password "<network-password>"`

`ssh` and `sshfs` are installed by default.

### Auto-Login

Run 
```bash
sudo systemctl edit getty@tty1.service
```

Then enter
```
[Service]
ExecStart=
ExecStart=-/sbin/agetty --noissue --autologin jetson %I $TERM
Type=idle
```

Make sure your username is actually `jetson` else you might need to reflash your sdcard!

## Supported boards:

- [Jetson nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
- [Jetson nano 2GB](https://developer.nvidia.com/embedded/jetson-nano-2gb-developer-kit)

## References

This work builds upon the learnings from this great post by pythops:

- https://pythops.com/post/create-your-own-image-for-jetson-nano-board.html
- https://github.com/pythops/jetson-nano-image

### Additional links

- https://developer.nvidia.com/embedded/linux-tegra
- https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/updating_jetson_and_host.html
- https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0CM0HA

## License

MIT
