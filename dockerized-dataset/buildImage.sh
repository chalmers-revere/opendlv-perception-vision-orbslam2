#!/bin/bash

cd $(dirname $0)

PROJ_ROOT=.

DOCKERFILE=Dockerfile.amd64
IMAGE_NAME=corroleaus/opendlv-orb2slam-assets

if docker build -f ${PROJ_ROOT}/${DOCKERFILE} -t ${IMAGE_NAME}:latest ${PROJ_ROOT}; then
    echo "                                                                                      "
    echo "######################################################################################"
    echo "To push the docker image to the registry, use the following command:"
    echo "docker push ${IMAGE_NAME}:latest"
    echo "######################################################################################"
    echo "                                                                                      "
else
    echo "                                                                                      "
    echo "######################################################################################"
    echo "                                                                                      "
    echo "                                  Build Failed...                                     "
    echo "                                                                                      "
    echo "######################################################################################"
    echo "                                                                                      "
fi