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

# Load sudo user nane
if [ ! -z $SUDO_USER ] ; then
    USER=$SUDO_USER
    HOSTNAME="$(cat /etc/hostname)"
fi

# Nanosaur configuration variables
CONFIG_FILE=".nanosaur.config"


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
    local DESKTOP=false
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            --desktop)
                DESKTOP=true
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

    local type_install=$( $DESKTOP && echo "desktop" || echo "robot" )
    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset}"
    echo " - ${bold}Home:${reset} ${green}$HOME${reset}"
    echo " - ${bold}Type:${reset} ${green}$type_install${reset}"
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

    # https://stackoverflow.com/questions/242538/unix-shell-script-find-out-which-directory-the-script-file-resides
    # Absolute path to this script, e.g. /home/user/bin/foo.sh
    SCRIPT=$(readlink -f "$0")
    # Absolute path this script is in, thus /home/user/bin
    SCRIPTPATH=$(dirname "$SCRIPT")
    if [ -d $SCRIPTPATH/bin ] ; then
        echo " - Copy nanosaur command in ${bold}${green}/usr/local/bin${reset}"
        sudo cp $SCRIPTPATH/bin/nanosaur /usr/local/bin/nanosaur
    #else
    #    echo " - ${bold}${green}Pull nanosaur command${reset} and copy in /usr/local/bin"
    #    sudo curl https://raw.githubusercontent.com/rnanosaur/nanosaur/master/nanosaur_bringup/scripts//bin/nanosaur -o /usr/local/bin/nanosaur
    #    sudo chmod +x /usr/local/bin/nanosaur
    fi

    if [ command -v pip &> /dev/null ] || [ command -v pip3 &> /dev/null ] ; then
        echo " - ${bold}${green}Install pip/pip3${reset}"
        sudo apt-get install -y python3-pip
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
        sudo apt-get install -y libffi-dev
        sudo apt-get install -y python-openssl
        sudo apt-get install libssl-dev
        # pip3 install --upgrade pip
        sudo pip3 install -U docker-compose
    fi

    # Disable sudo me
    sudo_me false
    # Run update function
    local THIS="$(pwd)"

    # Install basic packages on desktop
    if $DESKTOP ; then
        ROS_WS_NAME="nanosaur_ws"
        ROBOT_WORKSPACE=$HOME/$ROS_WS_NAME
        ROSINSTALL_FILE="https://raw.githubusercontent.com/rnanosaur/nanosaur/master/desktop.rosinstall"

        echo " - Write nanosaur configuration in ${bold}${green}$HOME/$CONFIG_FILE${reset}"
        #touch $HOME/$CONFIG_FILE
        echo "#!/bin/bash" > $HOME/$CONFIG_FILE
        # ROS main sources
        echo "export ROS_WS_NAME=$ROS_WS_NAME" >> $HOME/$CONFIG_FILE
        echo "export ROBOT_WORKSPACE=$ROBOT_WORKSPACE" >> $HOME/$CONFIG_FILE
        echo "export ROSINSTALL_FILE=$ROSINSTALL_FILE" >> $HOME/$CONFIG_FILE

        # Make nanosaur workspace
        if [ ! -d $ROBOT_WORKSPACE ] ; then
            echo " - Make Nanosaur ROS2 workspace folder in ${bold}${green}$ROBOT_WORKSPACE${reset}"
            mkdir -p $ROBOT_WORKSPACE/src
        else
            echo " - Nanosaur workspace folder already exist in ${bold}${yellow}$ROBOT_WORKSPACE${reset}"
        fi
        # Install wstool
        if ! command -v wstool &> /dev/null ; then
            echo " - ${bold}${green}Install wstool${reset}"
            sudo apt install python3-wstool -y
        fi
        # Initialize wstool
        # https://www.systutorials.com/docs/linux/man/1-wstool/
        if [ ! -f $ROBOT_WORKSPACE/src/.rosinstall ] ; then
            wstool init $ROBOT_WORKSPACE/src
        fi
        # Load wstool config
        wstool merge -t $ROBOT_WORKSPACE/src $ROSINSTALL_FILE
        # Update workspace
        wstool update -t $ROBOT_WORKSPACE/src
        # Build nanosaur
        nanosaur update
    fi

    # Return to main path
    cd $THIS

    # Download docker-compose and run the system
    if ! $DESKTOP ; then
        if [ ! -f $HOME/docker-compose.yml ] ; then
            echo " - Download Nanosaur docker-compose in ${bold}${yellow}$HOME/docker-compose.yml${reset}"
            # Download the docker-compose image and run
            curl https://raw.githubusercontent.com/rnanosaur/nanosaur/master/docker-compose.yml -o $HOME/docker-compose.yml
        fi
        # Run docker compose a daemon
        docker-compose -f $HOME/docker-compose.yml up -d
    fi

    if [ -f /var/run/reboot-required ] ; then
        # After install require reboot
        echo "${red}*** System Restart Required ***${reset}"
    fi
}

main $@
exit 0

# EOF
