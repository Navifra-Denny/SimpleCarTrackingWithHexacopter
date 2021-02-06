cmake_minimum_required(VERSION 3.5.0)
project(DroneServer)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ${CONTROL_ROOT}/DroneServer
  ${CONTROL_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
  ${CONTROL_ROOT}/AirLib/include
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} MavLinkCom)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
