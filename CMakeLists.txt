# CMakeLists.txt : CMake project for RoverBot
cmake_minimum_required(VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(RoverBot)

# Указываем, что исходники лежат в rover/
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rover)

set(EXTERN_DIR ${CMAKE_CURRENT_EXTERN_DIR}/extern)

# Список CPP-файлов
set(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/State.cpp
    ${SOURCE_DIR}/MathLib.cpp
    ${SOURCE_DIR}/Logger.cpp
    ${SOURCE_DIR}/NetworkCommand.cpp
    ${SOURCE_DIR}/NavigationMap.cpp
    ${SOURCE_DIR}/PathPlanner.cpp
    ${SOURCE_DIR}/StateHandlers.cpp
    ${SOURCE_DIR}/StateEnterHandlers.cpp
    ${SOURCE_DIR}/MissionController.cpp
    ${SOURCE_DIR}/UartDriver.cpp
    ${SOURCE_DIR}/StateMachine.cpp
)

# Заголовочные файлы (для IDE, не обязательны для сборки)
set(HEADERS
    ${SOURCE_DIR}/Robot.h
    ${SOURCE_DIR}/State.h
    ${SOURCE_DIR}/MathLib.h
    ${SOURCE_DIR}/Logger.h
    ${SOURCE_DIR}/Target.h
    ${SOURCE_DIR}/NetworkCommand.h
    ${SOURCE_DIR}/NavigationMap.h
    ${SOURCE_DIR}/PathPlanner.h
    ${SOURCE_DIR}/Event.h
    ${SOURCE_DIR}/EventQueue.h
    ${SOURCE_DIR}/StateHandlers.h
    ${SOURCE_DIR}/StateEnterHandlers.h
    ${SOURCE_DIR}/CommandTypes.h
    ${SOURCE_DIR}/MissionController.h
    ${SOURCE_DIR}/UartDriver.h
    ${SOURCE_DIR}/StateMachine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/stb_image.h
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/json.hpp
)

# Создаём исполняемый файл
add_executable(RoverBot ${SOURCES} ${HEADERS})

if(WIN32)
    target_compile_definitions(RoverBot PRIVATE _USE_MATH_DEFINES)
endif()

if(UNIX AND NOT APPLE)
    target_compile_definitions(RoverBot PRIVATE _LINUX)
    target_link_libraries(RoverBot PRIVATE pthread)
endif()

# Устанавливаем стандарт C++20
if(CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET RoverBot PROPERTY CXX_STANDARD 20)
    set_property(TARGET RoverBot PROPERTY CXX_STANDARD_REQUIRED ON)
endif()

# Проверяем существование директории extern/yaml-cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extern/yaml-cpp/CMakeLists.txt")
    # Подключаем yaml-cpp
    add_subdirectory(extern/yaml-cpp yaml-cpp-build)
    target_link_libraries(RoverBot PRIVATE yaml-cpp)
else()
    message(WARNING "yaml-cpp not found in extern/ directory")
endif()

# Копируем карты в директорию сборки
file(COPY ${CMAKE_SOURCE_DIR}/maps/
     DESTINATION ${CMAKE_BINARY_DIR}/maps)

# Добавляем rover/ и extern/ в include path
target_include_directories(RoverBot PRIVATE 
    ${SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
)