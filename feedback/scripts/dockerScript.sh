#!/bin/bash

docker kill $(docker container ls -q)
sleep 1
docker rm $(docker ps --filter status=exited -q)
sleep 1
docker run --device=/dev/video10 --device=/dev/video11 --device=/dev/video12 --device=/dev/video13 --net=host -p 8000:8000 -it --restart unless-stopped mpromonet/webrtc-streamer -a


