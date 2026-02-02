#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
export ZENOH_ROUTER_CONFIG_URI="$(readlink -f "$HERE")/zenoh_router_config.json5"
CREDENTIALS="$(readlink -f "$HERE")/credentials.txt"
export ZENOH_CONFIG_OVERRIDE='transport/auth/usrpwd/user="eval-router";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$CREDENTIALS'"'
