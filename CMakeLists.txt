cmake_minimum_required(VERSION 3.10)
project(robot)
find_package(Threads)

set(CMAKE_CXX_STANDARD 11)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


add_executable(robot main.cpp
        controller/ICodeurManager.cpp controller/ICodeurManager.h
        controller/MoteurManager.cpp controller/MoteurManager.h
        #Point.cpp Point.h
        controller/FakeCodeur.cpp controller/FakeCodeur.h controller/Odometry.cpp controller/Odometry.h  controller/PID.cpp controller/PID.h controller/Controller.cpp controller/Controller.h controller/MathUtils.cpp controller/MathUtils.h controller/Trajectory.cpp controller/Trajectory.h controller/Point.cpp controller/Point.h)
target_link_libraries(robot Threads::Threads)