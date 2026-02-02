#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
unset ZENOH_ROUTER_CONFIG_URI
CREDENTIALS="$(readlink -f "$HERE")/credentials.txt"
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7448"];routing/router/peers_failover_brokering=true;transport/auth/usrpwd/user="model-router";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="'$CREDENTIALS'"'
