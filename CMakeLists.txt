#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2024, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev:  $

cmake_minimum_required (VERSION 3.1)
project (ambf_openvr_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

include_directories(${AMBF_INCLUDE_DIRS})

link_directories(${AMBF_LIBRARY_DIRS})

add_definitions(${AMBF_DEFINITIONS})

add_subdirectory(/home/laminectomy/openvr bin)

set(OVR_HEADER_DIR
  ${OpenVRSDK_SOURCE_DIR}/headers
)

include_directories(${OVR_HEADER_DIR})

set(OVR_LIBS ${OpenVRSDK_SOURCE_DIR}/bin/linux64/libopenvr_api.so)

add_library(${CMAKE_PROJECT_NAME} SHARED plugins/object/${CMAKE_PROJECT_NAME}.cpp plugins/object/${CMAKE_PROJECT_NAME}.h)
target_link_libraries (${CMAKE_PROJECT_NAME} ${AMBF_LIBRARIES} ${OVR_LIBS})
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
