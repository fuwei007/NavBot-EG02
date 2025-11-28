include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_library(wheeltec_imu SHARED
        WheeltecImu.cpp)
target_link_libraries(wheeltec_imu spdlog::spdlog)

