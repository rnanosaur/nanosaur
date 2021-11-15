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
    # Detect platform
    local PLATFORM="$(uname -m)"
    local NANOSAUR_DATA='/opt/nanosaur'

    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    words=("${COMP_WORDS[@]}")
    cword=$COMP_CWORD

    case "$prev" in
        install)
            COMPREPLY=( $(compgen -W "--help -y dev" -- ${cur}) )
            return 0
        ;;
        update)
            if [[ $PLATFORM = "x86_64" ]]; then
                COMPREPLY=( $(compgen -W "rosinstall build -h" -- ${cur}) )
            else
                COMPREPLY=()
            fi
            return 0
        ;;
        run|up|start|stop|restart|logs|down)
            local services=$(docker-compose -f $NANOSAUR_DATA/docker-compose.yml ps --services)
            COMPREPLY=( $(compgen -W "$services" -- ${cur}) )
            return 0
        ;;
        help|info|distro|domain|network|config|wakeup|activate)
            COMPREPLY=()
            return 0
        ;;
    esac

    COMPREPLY=($(compgen -W "help info distro domain install update" "${COMP_WORDS[1]}"))
    # Add extra configurations
    if [[ $PLATFORM = "aarch64" ]]; then
        COMPREPLY+=($(compgen -W "run network config" "${COMP_WORDS[1]}"))
        # Docker
        COMPREPLY+=($(compgen -W "wakeup up start stop restart logs down" "${COMP_WORDS[1]}"))
    else
        COMPREPLY+=($(compgen -W "activate" "${cur}"))
    fi
    return 0
}

complete -F _dothis_completions nanosaur