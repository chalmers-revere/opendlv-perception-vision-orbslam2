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

#Get OS stuff
RUN apt-get update && apt-get -y install \
build-essential checkinstall cmake pkg-config yasm \
git gfortran mercurial \
libjpeg8-dev libjasper-dev libpng12-dev \
libtiff5-dev \
libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev \
libxine2-dev libv4l-dev \
libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev \
qt5-default libgtk2.0-dev libtbb-dev \
libatlas-base-dev \
libfaac-dev libmp3lame-dev libtheora-dev \
libvorbis-dev libxvidcore-dev \
libopencore-amrnb-dev libopencore-amrwb-dev \
x264 v4l-utils \
python-dev python-pip python3-dev python3-pip

RUN pip2 install -U pip numpy
RUN pip3 install -U pip numpy

# now install python libraries within this virtual environment
RUN pip install numpy scipy matplotlib scikit-image scikit-learn ipython

WORKDIR /tmp

RUN git clone https://github.com/opencv/opencv.git
RUN cd opencv && git checkout 3.3.1

RUN git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv_contrib && git checkout 3.3.1

RUN mkdir /tmp/opencv/build
WORKDIR /tmp/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
RUN make -j2
RUN make install
RUN sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
RUN ldconfig && rm -rf /tmp/opencv*

#RUN /usr/local/lib/python2.6/dist-packages/cv2.so
#RUN /usr/local/lib/python2.7/dist-packages/cv2.so
#RUN ## binary installed in site-packages
#RUN /usr/local/lib/python2.6/site-packages/cv2.so
#RUN /usr/local/lib/python2.7/site-packages/cv2.so

WORKDIR /tmp

RUN hg clone https://bitbucket.org/eigen/eigen && cd eigen && hg pull && hg update 3.2 && mkdir build && cd build && cmake .. && make -j4 install
RUN ldconfig && rm -rf /tmp/eigen
RUN git clone https://github.com/marbae/g2o && cd g2o && mkdir build && cd build && cmake -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF .. && make -j4 install
RUN ldconfig && rm -rf /tmp/g2o

#install Microservice
ADD . /opt/sources
RUN mkdir -p /opt/sources/build

WORKDIR /opt/sources/build

RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr ..
RUN make -j6
RUN make test
RUN make install
RUN ldconfig && rm -rf /opt/sources

FROM ubuntu:16.04
#MAINTAINER Christian Berger "christian.berger@gu.se"
#
##Start microservice
RUN apt-get update && apt-get -y install libglu1-mesa libpng12-dev libglib2.0-dev
WORKDIR /usr/bin
COPY --from=builder /usr/local/include/ /usr/local/include/
COPY --from=builder /usr/local/lib/ /usr/local/lib/
COPY --from=builder /usr/lib/ /usr/lib/
COPY --from=builder /usr/include/ /usr/include/
COPY --from=builder /usr/bin/opendlv-perception-vision-orbslam2 .
RUN ldconfig
CMD ["/usr/bin/opendlv-perception-vision-orbslam2-dest/bin/opendlv-perception-vision-orbslam2"]
##RUN apk update && apk add g++ #This is extremely ugly and needs to be fixed
