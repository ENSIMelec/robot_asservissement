cmake_minimum_required(VERSION 3.10)
project(robot)

set(CMAKE_CXX_STANDARD 14)

add_executable(robot main.cpp
        Asservissement.cpp Asservissement.h
        ICodeurManager.cpp ICodeurManager.h
        MoteurManager.cpp MoteurManager.h
       # SerialCodeurManager.cpp SerialCodeurManager.h
        Point.cpp Point.h
        FakeCodeur.cpp FakeCodeur.h controller/Odometry.cpp controller/Odometry.h controller/VelocityController.cpp controller/VelocityController.h controller/PositionController.cpp controller/PositionController.h controller/PID.cpp controller/PID.h controller/Controller.cpp controller/Controller.h)