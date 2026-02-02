#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
unset ZENOH_SESSION_CONFIG_URI
export ZENOH_CONFIG_OVERRIDE='transport/auth/usrpwd/user="participant";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$HERE/credentials.txt'"'
