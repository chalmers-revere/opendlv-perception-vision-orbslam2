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

cmake_minimum_required(VERSION 3.2)

project(opendlv-perception-vision-orbslam2)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set-v0.9.1.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.55.hpp)

################################################################################
# This project requires C++14 or newer.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Strip unneeded symbols from binaries.
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
# Build a static binary.
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
# Add further warning levels.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -D_XOPEN_SOURCE=700 \
    -D_FORTIFY_SOURCE=2 \
    -g \
    -O2 \
    -fstack-protector \
    -fomit-frame-pointer \
    -pipe \
    -pedantic -pedantic-errors \
    -Werror \
    -Weffc++ \
    -Wall -Wextra -Wshadow -Wdeprecated \
    -Wdiv-by-zero -Wfloat-equal -Wfloat-conversion -Wsign-compare -Wpointer-arith \
    -Wuninitialized -Wunreachable-code \
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-but-set-parameter -Wunused-but-set-variable \
    -Wunused-value -Wunused-variable -Wunused-result \
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")
# Threads are necessary for linking the resulting binaries as UDPReceiver is running in parallel.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

################################################################################
# Extract cluon-msc from cluon-complete.hpp.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/include/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${CLUON_COMPLETE})

################################################################################
# Generate opendlv-standard-message-set.{hpp,cpp} from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp-sources --cpp-add-include-file=opendlv-standard-message-set.hpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp-headers --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-msc)

# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

################################################################################
# Gather all object code first to avoid double compilation. This needs to be done for all added objects, can become very long? Why is this object and not static which works

################################################################################
# Including g2o in /usr/include
include_directories(SYSTEM /usr/include)
include_directories(SYSTEM /usr/local/include)
link_directories(SYSTEM /usr/lib) #Access g2o
link_directories(SYSTEM /usr/local/lib) #Access g2o

################################################################################
# Include openCV
set(CMAKE_PREFIX_PATH "/usr/local/opt/opencv3/share/OpenCV")

set(OpenCV_INCLUDE_DIRS "/usr/local/opt/opencv3/include")
set(OpenCV_LIBS "/usr/local/opt/opencv3/lib")

#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

#find_package(OpenCV REQUIRED)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()


################################################################################
# Link libs
set(LIBRARIES Threads::Threads g2o_core g2o_solver_eigen g2o_types_sim3 g2o_types_sba g2o_stuff opencv_core opencv_imgcodecs opencv_features2d opencv_imgproc opencv_highgui opencv_calib3d)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}-core STATIC 
${CMAKE_CURRENT_SOURCE_DIR}/src/selflocalization.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbkeyframedatabase.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/loopclosing.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbconverter.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orboptimizer.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/sim3solver.cpp
#${CMAKE_CURRENT_SOURCE_DIR}/src/imageextractor.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/mapping.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/pnpsolver.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbbowvector.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbconverter.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbdescriptor.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbextractor.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbfeaturevector.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbframe.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbinitializer.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbmappoint.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbmatcher.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbmap.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbvocabulary.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/tracking.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orbkeyframe.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/kittirunner.cpp
${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp)

################################################################################
# Create executable.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp) #Creates exe of the main .cpp (more like bin)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}-core ${LIBRARIES}) #Links the exe with Libraries like the objects added above and g2o

################################################################################
# Enable unit testing.
enable_testing()
add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/tests-opendlv-perception-vision-orbslam2.cpp)
target_link_libraries(${PROJECT_NAME}-runner ${PROJECT_NAME}-core ${LIBRARIES})
add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)

################################################################################
# Install executable.
install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})
