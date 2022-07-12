cmake_minimum_required(VERSION 3.22)
project(BubbleMAC_INFOCOM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

add_executable(BubbleMAC_INFOCOM main.cpp parameters.cpp parameters.h common.cpp common.h vehicle.cpp vehicle.h UpLocation.cpp UpLocation.h packets.cpp packets.h communication.cpp communication.h collision.cpp collision.h BubbleProtocol.cpp BubbleProtocol.h Protocol_80211.cpp Protocol_80211.h Protocol_VeMAC.cpp Protocol_VeMAC.h)
