cmake_minimum_required(VERSION 3.5.0)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

include_directories(
  ${CONTROL_ROOT}/AirLib/
  ${CONTROL_ROOT}/AirLib/include
  ${CONTROL_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
)

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${CONTROL_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} MavLinkCom)

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
