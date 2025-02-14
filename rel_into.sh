#!/usr/bin/env bash

###############################################################################
# Copyright 2023 Pattern Labs. Confidential and proprietary, do not distribute.
###############################################################################
DISPLAY_ENV="${DISPLAY}"
DISPLAY_NUM=`echo ${DISPLAY} | cut -d ':' -f 2 | cut -d '.' -f 1`
XAUTH_COOKIE=`xauth list | grep ":${DISPLAY_NUM}" | cut -d ' '  -f 2-`

xhost +local:root 1>/dev/null 2>&1

docker exec \
    -e DISPLAY=${DISPLAY_ENV} \
    -it apollo \
    bash -c "xauth add localhost:${DISPLAY_NUM} ${XAUTH_COOKIE} && /bin/bash"

xhost -local:root 1>/dev/null 2>&1
