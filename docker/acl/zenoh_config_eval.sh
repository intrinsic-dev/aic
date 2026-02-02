#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
CREDENTIALS="$(readlink -f "$HERE")/credentials.txt"
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7448"];transport/auth/usrpwd/user="eval";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$CREDENTIALS'"'
