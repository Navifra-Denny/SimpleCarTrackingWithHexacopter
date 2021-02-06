cmake_minimum_required(VERSION 3.2.2)
project(MavLinkTest)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../cmake-modules/CommonSetup.cmake")
CommonSetup()

SetupConsoleBuild()

include_directories(
  ${CONTROL_ROOT}/MavLinkCom/MavLinkTest
  ${CONTROL_ROOT}/MavLinkCom/include
  ${CONTROL_ROOT}/MavLinkCom/common_utils
)

    set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${CONTROL_ROOT}/MavLinkCom/${PROJECT_NAME}/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
			
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} MavLinkCom ${CXX_EXP_LIB})
