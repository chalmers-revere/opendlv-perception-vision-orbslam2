# opendlv-perception-vision-orbslam2

## build

### Docker

```bash buildImage.sh```

Run with dataset from KITTI:

```docker-compose pull```

```docker-compose up```

Visualizer will now be running on

[localhost:8081](http://localhost:8081)

### natively

In order to build and run natively, first install:

#### opencv

```
WORKDIR=$(pwd)
git clone https://github.com/opencv/opencv.git
cd opencv && git checkout 3.3.1
cd $WORKDIR
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib && git checkout 3.3.1
mkdir $WORKDIR/opencv/build && cd $WORKDIR/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
# adjust based on your CPU
make -j4
make install
echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf

#### g2o

git clone https://github.com/marbae/g2o && cd g2o && mkdir build && cd build && cmake -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF .. && make -j4 install

- eigen

hg clone https://bitbucket.org/eigen/eigen && cd eigen && hg pull && hg update 3.2 && mkdir build && cd build && cmake .. && make -j4 install


Now build opendlv-perception-vision-orbslam2:

```
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr ..
make -j4
make test
make install
```