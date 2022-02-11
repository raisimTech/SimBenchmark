########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -Wl,--no-as-needed -O0 -mtune=native -march=native -g")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native -fPIC")

########################################################################################################################
# executable
find_package(raisim CONFIG REQUIRED)

add_executable(raiAnymalPDBenchmark AnymalPDBenchmark.cpp)
target_link_libraries(raiAnymalPDBenchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(raiAnymalMomentumBenchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
target_link_libraries(raiAnymalEnergyBenchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(raiRollingBenchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiKaplaBenchmark KaplaBenchmark.cpp)
target_link_libraries(raiKaplaBenchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(rai666Benchmark 666Benchmark.cpp)
target_link_libraries(rai666Benchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiElastic666Benchmark Elastic666Benchmark.cpp)
target_link_libraries(raiElastic666Benchmark raisim::raisim benchmark yaml-cpp pthread)

add_executable(raiBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(raiBouncingBenchmark raisim::raisim benchmark yaml-cpp pthread)