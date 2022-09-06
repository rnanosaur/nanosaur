# 🦕 nanosaur

[![Discord](https://img.shields.io/discord/797461428646707211)](https://discord.gg/YvxjxEFPkb) [![GitHub Org's stars](https://img.shields.io/github/stars/rnanosaur?style=social)](https://github.com/rnanosaur) [![Twitter Follow](https://img.shields.io/twitter/follow/raffaello86?style=social)](https://twitter.com/raffaello86) [![robo.panther](https://img.shields.io/badge/Follow:-robo.panther-E4405F?style=social&logo=instagram)](https://www.instagram.com/robo.panther/)

**nanosaur** The smallest [NVIDIA Jetson](https://developer.nvidia.com/buy-jetson) dinosaur robot, **open-source**, fully **3D printable**, based on [**ROS2**](https://www.ros.org/) & [**Isaac ROS**](https://developer.nvidia.com/isaac-ros-gems).

<small>Designed & made by [Raffaello Bonghi](https://rnext.it)</small>

[![nanosaur](https://nanosaur.ai/assets/images/banner.jpg)](https://nanosaur.ai)

Meet nanosaur:
* 🦕 Website: [nanosaur.ai](https://nanosaur.ai)
* 🦄 Do you need an help? [Discord](https://discord.gg/YvxjxEFPkb)
* 🧰 For technical details follow [wiki](https://github.com/rnanosaur/nanosaur/wiki)
* 🐳 nanosaur [Docker hub](https://hub.docker.com/u/nanosaur)
* ⁉️ Something wrong? Open an [issue](https://github.com/rnanosaur/nanosaur/issues)

If you are looking for an NVIDIA Jetson Nano alternative, you can user for nanosaur [SEEED Studio - reComputer J1010](https://www.seeedstudio.com/Jetson-10-1-H0-p-5335.html)

# CI & CD

**Latest** = ROS2 **_foxy_** at latest tag released

| 🏗️ CI            | latest* | foxy | galactic |
|:-------------:|:-------:|:----:|:--------:|
| 🧠 [core](https://github.com/rnanosaur/nanosaur.git) | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml/badge.svg?branch=master)](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml) | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml/badge.svg?branch=foxy)](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml) | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml/badge.svg?branch=galactic)](https://github.com/rnanosaur/nanosaur/actions/workflows/docker-image.yml) |
| 🖼️ [perception](https://github.com/rnanosaur/nanosaur_perception.git)   | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml/badge.svg?branch=main)](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml) | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml/badge.svg?branch=foxy)](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml) | [![Docker Builder CI](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml/badge.svg?branch=galactic)](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml) |

🏗️ **nanosaur_robot CI** [![nanosaur_robot rebuild](https://github.com/rnanosaur/nanosaur_robot/actions/workflows/root-update.yml/badge.svg)](https://github.com/rnanosaur/nanosaur_robot/actions/workflows/root-update.yml)

-------------------------------------

| 🐳 Docker        | latest* | foxy | galactic | Pulls |
|:-------------:|:-------:|:----:|:--------:|:-----:|
| 🧠 [core](https://github.com/rnanosaur/nanosaur.git) | [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/nanosaur/latest)](https://hub.docker.com/r/nanosaur/nanosaur) | [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/nanosaur/foxy)](https://hub.docker.com/r/nanosaur/nanosaur) | [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/nanosaur/galactic)](https://hub.docker.com/r/nanosaur/nanosaur) | [![Docker Pulls](https://img.shields.io/docker/pulls/nanosaur/nanosaur)](https://hub.docker.com/r/nanosaur/nanosaur) |
| 🖼️ [perception](https://github.com/rnanosaur/nanosaur_perception.git)    |  [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/latest)](https://hub.docker.com/r/nanosaur/perception) <br/> zed <br/> [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/latest-zed)](https://hub.docker.com/r/nanosaur/perception) <br/> realsense <br/> [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/latest-realsense)](https://hub.docker.com/r/nanosaur/perception) | [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/foxy)](https://hub.docker.com/r/nanosaur/perception) <br/> zed <br/> [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/foxy-zed)](https://hub.docker.com/r/nanosaur/perception) <br/> realsense <br/> [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/foxy-realsense)](https://hub.docker.com/r/nanosaur/perception) | [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/nanosaur/perception/galactic)](https://hub.docker.com/r/nanosaur/perception) | [![Docker Pulls](https://img.shields.io/docker/pulls/nanosaur/perception)](https://hub.docker.com/r/nanosaur/perception) |

# License

* All code is Under license [MIT](LICENSE)
* All stl files included in **nanosaur_description/meshes** are under [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg

For more information about this project please follow [nanosaur.ai/about](https://nanosaur.ai/about/#license)
