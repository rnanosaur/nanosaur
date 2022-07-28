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

# https://iridakos.com/programming/2018/03/01/bash-programmable-completion-tutorial
_dothis_completions()
{
    # Load platform
    # - aarch64 = NVIDIA Jetson
    # - x86_64 = Desktop
    # PLATFORM="$(uname -m)"
    local PLATFORM="desktop"
    if [ -f /etc/nv_tegra_release ] ; then
        PLATFORM="robot"
    fi

    local NANOSAUR_DATA='/opt/nanosaur'

    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    words=("${COMP_WORDS[@]}")
    cword=$COMP_CWORD

    case "$prev" in
        install)
            COMPREPLY=( $(compgen -W "developer --force --help" -- ${cur}) )
            return 0
        ;;
        update)
            if [[ $PLATFORM = "desktop" ]]; then
                COMPREPLY=($(compgen -W "rosinstall --help" -- ${cur}))
            else
                COMPREPLY=($(compgen -W "--clean" -- ${cur}))
            fi
            return 0
        ;;
        simulation)
            COMPREPLY=( $(compgen -W "run set --help" -- ${cur}) )
            return 0
        ;;
        build)
            COMPREPLY=( $(compgen -W "clean --help --verbose" -- ${cur}) )
            return 0
        ;;
        clean)
            if [[ $PLATFORM = "robot" ]]; then
                COMPREPLY=($(compgen -W "-f" -- ${cur}))
            else
                COMPREPLY=()
            fi
            return 0
        ;;
        run|start|restart|stop|up|logs|down|top|exec|rm)
            local services=$(docker-compose -f $NANOSAUR_DATA/docker-compose.yml ps --services)
            COMPREPLY=( $(compgen -W "$services" -- ${cur}) )
            return 0
        ;;
        help|info|cover|config|dds|domain|distro|network|wakeup)
            COMPREPLY=()
            return 0
        ;;
    esac

    COMPREPLY=($(compgen -W "help info cover config dds domain install update" "${COMP_WORDS[1]}"))
    # Add extra configurations
    if [[ $PLATFORM = "robot" ]]; then
        COMPREPLY+=($(compgen -W "distro network wakeup down clean" "${COMP_WORDS[1]}"))
        # Docker
        COMPREPLY+=($(compgen -W "start restart stop up logs top rm exec" "${COMP_WORDS[1]}"))
    else
        COMPREPLY+=($(compgen -W "build branch perception simulation teleop" "${cur}"))
    fi
    return 0
}

complete -F _dothis_completions nanosaur