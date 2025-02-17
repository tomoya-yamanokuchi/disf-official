#!/bin/bash

docker build -f docker/Dockerfile --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t docker_disf:latest .

