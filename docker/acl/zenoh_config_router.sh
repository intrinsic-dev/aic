#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
export ZENOH_ROUTER_CONFIG_URI="$HERE/zenoh_router_config.json5"
export ZENOH_CONFIG_OVERRIDE='transport/auth/usrpwd/user="router";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$HERE/credentials.txt'"'
