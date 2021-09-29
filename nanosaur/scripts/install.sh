#!/bin/bash
# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

# Load platform
# - aarch64 = NVIDIA Jetson
# - x86_64 = Desktop
PLATFORM="$(uname -m)"

# Load sudo user nane
if [ ! -z $SUDO_USER ] ; then
    USER=$SUDO_USER
    HOSTNAME="$(cat /etc/hostname)"
fi

# Nanosaur configuration variables
CONFIG_FILE=".nanosaur.config"
NANOSAUR_DATA='/opt/nanosaur'
ROS_WS_NAME="nanosaur_ws"
NANOSAUR_WORKSPACE=$HOME/$ROS_WS_NAME

usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    echo "${bold}${green}nanosaur installer.${reset} This script install all dependencies. Use this script to setup your host."
    echo "${bold}${red}Do not use${reset} to setup a docker image"
    echo "Usage:"
    echo "$0 [options]"
    echo "options,"
    echo "   -h|--help            | This help"
    echo "   -s|--silent          | Run this script silent"
    echo "   --desktop            | Install nanosaur for desktop"
}


# Modules check sudo
# https://serverfault.com/questions/266039/temporarily-increasing-sudos-timeout-for-the-duration-of-an-install-script
sudo_me()
{
    local start=$1
    # write sudo me file
    local sudo_stat="/tmp/nanosaur_sudo_status"
    # No sudo loop
    if $start ; then
        touch $sudo_stat
        # Loop script
        while [ -f $sudo_stat ]; do
            # echo "checking $$ ...$(date)"
            sudo -v
            sleep 5
        done &
    else
        if [ -f $sudo_stat ] ; then
            rm $sudo_stat
        fi
    fi
}


main()
{
    local SILENT=false
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -s|--silent)
                SILENT=true
                ;;
            *)
                usage "[ERROR] Unknown option: $1"
                exit 1
            ;;
        esac
            shift 1
    done

	# Check run in sudo
    if [[ `id -u` -eq 0 ]] ; then 
        echo "${red}Please don't run as root${reset}"
        exit 1
    fi

    local type_install="desktop"
    if [[ $PLATFORM = "aarch64" ]] ; then
        type_install="robot"
    fi
    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset}"
    echo " - ${bold}Home:${reset} ${green}$HOME${reset}"
    echo " - ${bold}Install on:${reset} ${green}$type_install${reset}"
    echo "---------------------------"

    while ! $SILENT; do
        read -p "Do you wish to install nanosaur config? [Y/n] " yn
            case $yn in
                [Yy]* ) # Break and install jetson_stats 
                        break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v
    sudo_me true

    if [ ! -d $NANOSAUR_DATA ] ; then
        echo " - ${bold}${green}Make nanosaur folder in $NANOSAUR_DATA${reset}"
        # Build nanosaur folder structure
        # - /opt/nanosaur
        # -      /param [ ros2 parameter folder ]
        sudo mkdir -p $NANOSAUR_DATA
        sudo chown $USER:$USER $NANOSAUR_DATA
        mkdir -p "$NANOSAUR_DATA/param"
    fi

    # Check if exist the nanosaur workspace
    if [ -d $NANOSAUR_WORKSPACE ] ; then
        if [ ! -L $NANOSAUR_DATA/nanosaur ] ; then
            echo " - Link nanosaur command in ${bold}${green}$NANOSAUR_DATA${reset}"
            ln -s $NANOSAUR_WORKSPACE/src/nanosaur/scripts/bin $NANOSAUR_DATA/nanosaur
        fi
    else
        echo " - ${bold}${green}Pull nanosaur command${reset} and copy in $NANOSAUR_DATA"
        curl https://raw.githubusercontent.com/rnanosaur/nanosaur/master/nanosaur/scripts/bin/nanosaur -o $NANOSAUR_DATA/nanosaur
        chmod +x $NANOSAUR_DATA/nanosaur
    fi
    # link nanosaur script to nanosaur path
    if [ ! -f /usr/local/bin/nanosaur ] ; then
        echo " - Link nanosaur command in ${bold}${green}/usr/local/bin${reset}"
        sudo ln -s $NANOSAUR_DATA/nanosaur /usr/local/bin/nanosaur
    fi

    # Installer for NVIDIA Jetson platform
    if [[ $PLATFORM = "aarch64" ]] ; then

        if ! command -v pip3 &> /dev/null ; then
            echo " - ${bold}${green}Install pip/pip3${reset}"
            sudo apt-get install -y python3-pip
        fi

        # Check if is installed nano
        if ! command -v nano &> /dev/null ; then
            echo " - ${bold}${green}Install nano${reset}"
            sudo apt-get install -y nano
        fi

        # Check if is installed jtop
        if ! command -v jtop &> /dev/null ; then
            echo " - ${bold}${green}Install/Update jetson-stats${reset}"
            sudo -H pip3 install -U jetson-stats
        fi

        if ! getent group docker | grep -q "\b$USER\b" ; then
            echo " - Add docker permissions to ${bold}${green}user=$USER${reset}"
            sudo usermod -aG docker $USER
        fi

        # Check if is installed docker-compose
        if ! command -v docker-compose &> /dev/null ; then
            echo " - ${bold}${green}Install docker-compose${reset}"
            sudo apt-get install -y libffi-dev python-openssl libssl-dev
            sudo -H pip3 install -U pip
            sudo pip3 install -U docker-compose
        fi

        if [ -d $NANOSAUR_WORKSPACE ] ; then
            if [ ! -L $NANOSAUR_DATA/docker-compose.yml ] ; then
                echo " - ${bold}${green}Link Nanosaur docker-compose${reset}" >&2
                ln -s  $NANOSAUR_WORKSPACE/src/nanosaur/docker-compose.yml $NANOSAUR_DATA/docker-compose.yml
            fi
        else
            # Download latest version nanosaur docker-compose
            echo " - ${bold}${green}Download Nanosaur docker-compose${reset}" >&2
            # Download the docker-compose image and run
            curl https://raw.githubusercontent.com/rnanosaur/nanosaur/master/docker-compose.yml -o $NANOSAUR_DATA/docker-compose.yml
        fi

        # Run docker compose a daemon
        echo " - ${bold}${green}Start nanosaur docker-compose${reset}"
        sudo docker-compose -f $NANOSAUR_DATA/docker-compose.yml up -d

    fi

    # Disable sudo me
    sudo_me false
    # Run update function
    local THIS="$(pwd)"

    # Install basic packages on desktop
    if [[ $PLATFORM = "x86_64" ]] ; then
        ROSINSTALL_FILE="https://raw.githubusercontent.com/rnanosaur/nanosaur/master/nanosaur/rosinstall/desktop.rosinstall"

        # Make nanosaur workspace
        if [ ! -d $NANOSAUR_WORKSPACE ] ; then
            echo " - Make Nanosaur ROS2 workspace folder in ${bold}${green}$NANOSAUR_WORKSPACE${reset}"
            mkdir -p $NANOSAUR_WORKSPACE/src
        else
            echo " - Nanosaur workspace folder already exist in ${bold}${yellow}$NANOSAUR_WORKSPACE${reset}"
        fi
        # Install wstool
        if ! command -v wstool &> /dev/null ; then
            echo " - ${bold}${green}Install wstool${reset}"
            sudo apt install python3-wstool -y
        fi
        # Initialize wstool
        # https://www.systutorials.com/docs/linux/man/1-wstool/
        if [ ! -f $NANOSAUR_WORKSPACE/src/.rosinstall ] ; then
            wstool init $NANOSAUR_WORKSPACE/src
        fi
        # Build nanosaur
        nanosaur update --rosinstall
    fi

    # Return to main path
    cd $THIS

    if [ -f /var/run/reboot-required ] ; then
        # After install require reboot
        echo "${red}*** System Restart Required ***${reset}"
    fi
}

main $@
exit 0

# EOF
