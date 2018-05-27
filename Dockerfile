# Copyright (C) 2018  Christian Berger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Part to build opendlv-perception-vision-orbslam2.
FROM ubuntu:16.04 as builder
MAINTAINER Christian Berger "christian.berger@gu.se"

RUN apt-get update && apt-get -y install \
build-essential \
git mercurial \
cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev \
libglu1-mesa libpng12-dev libglib2.0-dev

WORKDIR /tmp

RUN git clone https://github.com/opencv/opencv.git && \
cd opencv && \
git checkout 3.3.1

RUN mkdir /tmp/opencv/build
WORKDIR /tmp/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local ..
RUN make -j4 && \
make install && \
sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && \
ldconfig && \
rm -rf /tmp/opencv*

WORKDIR /tmp

RUN hg clone https://bitbucket.org/eigen/eigen && cd eigen && hg pull && hg update 3.2 && mkdir build && cd build && cmake .. && make -j4 install
RUN ldconfig && rm -rf /tmp/eigen
RUN git clone https://github.com/marbae/g2o && cd g2o && mkdir build && cd build && cmake -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF .. && make -j4 install
RUN ldconfig && rm -rf /tmp/g2o

#install Microservice
ADD . /opt/sources
RUN mkdir -p /opt/sources/build
WORKDIR /opt/sources/build

RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr .. && \
make -j6 && \
make test && \
make install && \
ldconfig && \
rm -rf /opt/sources
CMD ["/usr/bin/opendlv-perception-vision-orbslam2"]
