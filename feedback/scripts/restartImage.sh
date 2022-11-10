#!/bin/bash

sudo systemctl stop merop-v4l2loopback
sleep 2
sudo systemctl start merop-v4l2loopback
