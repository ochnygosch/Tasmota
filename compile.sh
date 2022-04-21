#!/bin/bash
docker run -ti --rm \
-v $(pwd):/tasmota \
-u $UID:$GID blakadder/docker-tasmota \
-e tasmota-DE
