cmake_minimum_required(VERSION 3.16)
project(Project_MOT)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include third_party/hungarian-algorithm-cpp)

find_library(ONNXRUNTIME_LIB onnxruntime HINTS $ENV{ONNXRUNTIME_ROOT}/lib)

add_executable(mot_tracker
    src/main.cpp
    src/tracker.cpp
    src/gt_detector.cpp
    src/pointpillars_detector.cpp
    src/pointpillars_runtime_detector.cpp
    src/associator.cpp
    src/linear_kf.cpp
    src/track.cpp
    src/cost_function.cpp
    third_party/hungarian-algorithm-cpp/Hungarian.cpp
)

target_include_directories(mot_tracker PRIVATE $ENV{ONNXRUNTIME_ROOT}/include)

target_link_libraries(mot_tracker
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    ${ONNXRUNTIME_LIB}
)
