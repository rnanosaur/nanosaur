# 🦕 nanosaur
<!-- SOCIAL START -->
[![Sponsor](https://img.shields.io/badge/Sponsor-30363D?logo=GitHub-Sponsors&logoColor=#white)](https://github.com/sponsors/rbonghi) [![Discord](https://img.shields.io/discord/797461428646707211?style=social&logo=discord&label=Discord)](https://discord.gg/rCHgeUpUj9) [![GitHub rnanosaur stars](https://img.shields.io/github/stars/rnanosaur?style=social)](https://github.com/rnanosaur) [![GitHub rbonghi followers](https://img.shields.io/github/followers/rbonghi?label=rbonghi)](https://github.com/rbonghi) [![LinkedIn](https://img.shields.io/badge/LinkedIn:-raffaello--bonghi-0077B5?style=social)](https://www.linkedin.com/in/raffaello-bonghi) [![robo.panther](https://img.shields.io/badge/Follow:-robo.panther-E4405F?style=social&logo=instagram)](https://www.instagram.com/robo.panther)
<!-- SOCIAL END -->
<!-- INTRO START -->
**nanosaur** The smallest [NVIDIA Jetson](https://developer.nvidia.com/buy-jetson) dinosaur robot, **open-source**, fully **3D printable**, based on [**ROS 2**](https://www.ros.org/) & [**Isaac ROS**](https://developer.nvidia.com/isaac-ros-gems).

*Designed & made by [Raffaello Bonghi](https://rnext.it)*

[![nanosaur](https://nanosaur.ai/assets/images/banner.jpg)](https://nanosaur.ai)

Meet nanosaur:

* 🦕 Website: [nanosaur.ai](https://nanosaur.ai)
* 🦄 Do you need any help? [Discord](https://discord.gg/rCHgeUpUj9)
* 🧰 For technical details, follow [wiki](https://github.com/rnanosaur/nanosaur/wiki)
* 🐳 nanosaur [Docker Hub](https://hub.docker.com/u/nanosaur)
* ⁉️ Something wrong? Open an [issue](https://github.com/rnanosaur/nanosaur/issues)
<!-- INTRO END -->
<!-- CI START -->
## CI & CD

List of all nanosaur software release and CI/CD status.

| 📦 package | 🏗️ CI | Downloads | Health |
|:----------:|:-----:|:---------:|:------:|
| [![GitHub Release](https://img.shields.io/github/v/release/rnanosaur/nanosaur?label=nanosaur)](https://github.com/rnanosaur/nanosaur/releases) | [![Release nanosaur](https://github.com/rnanosaur/nanosaur/actions/workflows/release.yml/badge.svg)](https://github.com/rnanosaur/nanosaur/actions/workflows/release.yml) | [![GitHub forks](https://img.shields.io/github/forks/rnanosaur/nanosaur)](https://github.com/rnanosaur/nanosaur) | --- |
| [![PyPI - Version](https://img.shields.io/pypi/v/nanosaur?label=nanosaur-cli)](https://badge.fury.io/py/nanosaur) | [![Publish Python Package](https://github.com/rnanosaur/nanosaur_cli/actions/workflows/release.yml/badge.svg)](https://github.com/rnanosaur/nanosaur_cli/actions/workflows/release.yml) | [![PyPI - Downloads](https://img.shields.io/pypi/dm/nanosaur)](https://pypistats.org/packages/nanosaur) | [![nanosaur](https://snyk.io/advisor/python/nanosaur/badge.svg)](https://snyk.io/advisor/python/nanosaur) |
| [![Website Badge](https://img.shields.io/badge/Website-green)](https://nanosaur.ai) | [![pages-build-deployment](https://github.com/rnanosaur/rnanosaur.github.io/actions/workflows/pages/pages-build-deployment/badge.svg)](https://github.com/rnanosaur/rnanosaur.github.io/actions/workflows/pages/pages-build-deployment) | --- | --- |


### Docker Images

List of Docker images for nanosaur sorted by category and architecture supported. (The docker pulls are related to the Docker Hub repository)

| Category   | Architecture | Image | Pulls | Size |
|:----------:|:------------:|-------|-------|------|
| 🦕 | 🖥️ x86_64 | [nanosaur/nanosaur:simulation](https://hub.docker.com/r/nanosaur/nanosaur) | [![nanosaur/nanosaur Pulls](https://img.shields.io/docker/pulls/nanosaur/nanosaur)](https://hub.docker.com/r/nanosaur/nanosaur) | [![nanosaur/nanosaur Size](https://img.shields.io/docker/image-size/nanosaur/nanosaur/simulation)](https://hub.docker.com/r/nanosaur/nanosaur) |
| 🦕 | 🕹️ Jetson | [nanosaur/nanosaur:robot](https://hub.docker.com/r/nanosaur/nanosaur) | [![nanosaur/nanosaur Pulls](https://img.shields.io/docker/pulls/nanosaur/nanosaur)](https://hub.docker.com/r/nanosaur/nanosaur) | [![nanosaur/nanosaur Size](https://img.shields.io/docker/image-size/nanosaur/nanosaur/robot)](https://hub.docker.com/r/nanosaur/nanosaur) |
| 🖼️ | 🖥️ x86_64 | [nanosaur/perception:simulation](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Pulls](https://img.shields.io/docker/pulls/nanosaur/perception)](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Size](https://img.shields.io/docker/image-size/nanosaur/perception/simulation)](https://hub.docker.com/r/nanosaur/perception) |
| 🖼️ | 🕹️ Jetson | [nanosaur/perception:realsense](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Pulls](https://img.shields.io/docker/pulls/nanosaur/perception)](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Size](https://img.shields.io/docker/image-size/nanosaur/perception/realsense)](https://hub.docker.com/r/nanosaur/perception) |
| 🖼️ | 🕹️ Jetson | [nanosaur/perception:zed](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Pulls](https://img.shields.io/docker/pulls/nanosaur/perception)](https://hub.docker.com/r/nanosaur/perception) | [![nanosaur/perception Size](https://img.shields.io/docker/image-size/nanosaur/perception/zed)](https://hub.docker.com/r/nanosaur/perception) |
| 👨‍💻 | 🖥️ x86_64 | [nanosaur/simulation:gazebo](https://hub.docker.com/r/nanosaur/simulation) | [![nanosaur/simulation Pulls](https://img.shields.io/docker/pulls/nanosaur/simulation)](https://hub.docker.com/r/nanosaur/simulation) | [![nanosaur/simulation Size](https://img.shields.io/docker/image-size/nanosaur/simulation/gazebo)](https://hub.docker.com/r/nanosaur/simulation) |
| 👨‍💻 | 🖥️ x86_64 | [nanosaur/simulation:isaac-sim](https://hub.docker.com/r/nanosaur/simulation) | [![nanosaur/simulation Pulls](https://img.shields.io/docker/pulls/nanosaur/simulation)](https://hub.docker.com/r/nanosaur/simulation) | [![nanosaur/simulation Size](https://img.shields.io/docker/image-size/nanosaur/simulation/isaac-sim)](https://hub.docker.com/r/nanosaur/simulation) |

<!-- CI END -->
<!-- DISTRIBUTION START -->
## Distribution

This section displays the current nanosaur version and the map of the software version in use.

The latest release is in bold.

| Version | Branch | ROS | Isaac ROS Release | Isaac ROS Distro | Isaac Sim |
| --- | --- | --- | --- | --- | --- |
| **2.0.0** | **nanosaur2** | **humble** | **release-3.2** | **ros2_humble** | **>=4.1, <=4.5** |
<!-- DISTRIBUTION END -->
<!-- LICENSE START -->
## License

* All code is Under license [MIT](LICENSE)

For more information about this project, please follow [nanosaur.ai/about](https://nanosaur.ai/about/#license)
<!-- LICENSE END -->
### License mesh

* All stl files included in **nanosaur_description/meshes** are under [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
<!-- CREDITS START -->
## Contact Information

If you have any questions or inquiries, please contact us at [raffaello@nanosaur.ai](mailto:raffaello@nanosaur.ai).
<!-- CREDITS END -->
