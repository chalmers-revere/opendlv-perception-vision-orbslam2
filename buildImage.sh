# docker-compose.yml - Composition of micro-services to run OpenDLV software.
# Copyright (C) 2016 Christian Berger
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# Date: 2016-10-16

#!/bin/bash

cd $(dirname $0)

PROJ_ROOT=.

DOCKERFILE=Dockerfile.amd64
IMAGE_NAME=opendlv-perception-vision-orbslam2-amd64

if docker build -f ${PROJ_ROOT}/${DOCKERFILE} -t chalmersrevere/${IMAGE_NAME}:latest ${PROJ_ROOT}; then
    echo "                                                                                      "
    echo "######################################################################################"
    echo "To push the docker image to the registry, use the following command:"
    echo "docker push chalmersrevere/${IMAGE_NAME}:latest"
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
