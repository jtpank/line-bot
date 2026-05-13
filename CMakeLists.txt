cmake_minimum_required(VERSION 3.20)

project(paint_robot_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_program(CARGO_EXECUTABLE cargo REQUIRED)

set(DORA_C_API_TARGET_DIR ${CMAKE_BINARY_DIR}/dora-c-api-target)
set(DORA_C_API_LIB ${DORA_C_API_TARGET_DIR}/release/libdora_node_api_c.a)

add_custom_command(
    OUTPUT ${DORA_C_API_LIB}
    COMMAND ${CARGO_EXECUTABLE} build
            --manifest-path ${CMAKE_SOURCE_DIR}/dora-c-api/Cargo.toml
            --release
            --target-dir ${DORA_C_API_TARGET_DIR}
    DEPENDS
            ${CMAKE_SOURCE_DIR}/dora-c-api/Cargo.toml
            ${CMAKE_SOURCE_DIR}/dora-c-api/src/lib.rs
    VERBATIM
)

add_custom_target(dora_c_api ALL DEPENDS ${DORA_C_API_LIB})

add_library(sim_common
    src/common/boundaries.cc
    src/common/dora_node.cc
    src/common/geometry.cc
    src/common/messages.cc
    src/common/paint_arm.cc
)

target_include_directories(sim_common PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

function(add_sim_node target source)
    add_executable(${target} ${source})
    add_dependencies(${target} dora_c_api)
    target_link_libraries(${target} PRIVATE sim_common ${DORA_C_API_LIB})
    target_include_directories(${target} PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    if(APPLE)
        target_link_libraries(${target} PRIVATE
            "-framework CoreServices"
            "-framework Security"
            resolv
            pthread
            c
            m
        )
    elseif(UNIX)
        target_link_libraries(${target} PRIVATE m rt dl pthread)
    endif()
endfunction()

add_sim_node(map_server src/nodes/map_server.cc)
add_sim_node(planner src/nodes/planner.cc)
add_sim_node(controller src/nodes/controller.cc)
add_sim_node(vehicle_sim src/nodes/vehicle_sim.cc)
add_sim_node(gps_sim src/nodes/gps_sim.cc)
add_sim_node(imu_sim src/nodes/imu_sim.cc)
add_sim_node(state_estimator src/nodes/state_estimator.cc)
add_sim_node(paint_tracker src/nodes/paint_tracker.cc)
add_sim_node(monitor src/nodes/monitor.cc)
