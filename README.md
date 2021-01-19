# :sauropod: nanosaur

[![Docker Builder CI](https://github.com/rnanosaur/nanosaur/workflows/Docker%20Builder%20CI/badge.svg)](https://github.com/rnanosaur/nanosaur/actions?query=workflow%3A%22Docker+Builder+CI%22)

NanoSaur is a little tracked robot ROS2 enabled, made for an NVIDIA Jetson Nano

* Website: [nanosaur.ai](https://nanosaur.ai)
* Do you need an help? [Discord](https://discord.gg/NSrC52P5mw)
* For technical details follow [wiki](https://github.com/rnanosaur/nanosaur/wiki)
* Something wrong? Open an [issue](https://github.com/rnanosaur/nanosaur/issues)

## Install nanosaur system
```
git clone https://github.com/rnanosaur/nanosaur.git
sudo bash nanosaur/nanosaur_bringup/scripts/install.sh
```

After the installation please **reboot** the jetson board

## Run nanosaur

This script start the docker-compose

```
cd nanosaur/
docker-compose up -d
```

# Develop

This following part is to develop with nanosaur

##  Build Docker

Make ROS2 foxy jetson-container

```
git clone --branch patch-1 https://github.com/rbonghi/jetson-containers.git
./jetson-containers/scripts/docker_build_ros.sh foxy
```

After foxy build, build the nanosaur docker for jetson

```
cd nanosaur
docker build -f Dockerfile.dev -t nanosaur:l4t-r32.4.4 .
```

## Run docker container

https://answers.ros.org/question/358453/ros2-docker-multiple-hosts/

```
docker run --runtime nvidia -it --rm  --network host --device /dev/i2c-1 -v /tmp/argus_socket:/tmp/argus_socket -v $HOME/nanosaur:/opt/ros_ws/src/nanosaur nanosaur:l4t-r32.4.4 bash
```

## Detect I2C devices

Install I2C tools and dectect all devices

```
sudo apt-get install -y python-smbus
sudo apt-get install -y i2c-tools
```

```
sudo i2cdetect -y -r 1
```

Devices:
* **3C** left Display
* **3D** right display
* **60** motor driver

## Camera fail

If the camera fail do:

```
sudo systemctl restart nvargus-daemon
```

And rerun the docker container