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
    if [[ `id -u` -ne 0 ]] ; then 
        echo "${red}Please run as root${reset}"
        exit 1
    fi

    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset}"
    echo " - ${bold}Home:${reset} ${green}$HOME${reset}"
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

    sudo apt-get install -y python3-pip
    echo " - ${bold}${green}Install jetson-stats${reset}"
    sudo -H pip3 install -U jetson-stats

    echo " - ${bold}${green}Add docker permissions to user=$user${reset}"
    sudo usermod -aG docker $USER

    echo " - ${bold}${green}Install docker-compose${reset}"
    sudo apt-get install -y libffi-dev
    sudo apt-get install -y python-openssl
    sudo apt-get install libssl-dev
    pip3 install --upgrade pip
    pip3 install -U docker-compose

    if [ -f /var/run/reboot-required ] ; then
        # After install require reboot
        echo "${red}*** System Restart Required ***${reset}"
    fi
}

main $@
exit 0

# EOF
