#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
CREDENTIALS="$(readlink -f "$HERE")/model-credentials.txt"
export ZENOH_CONFIG_OVERRIDE='transport/auth/usrpwd/user="model";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$CREDENTIALS'"'
