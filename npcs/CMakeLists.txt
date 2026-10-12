# DiamondNav's build of open.mp's NPC component, with NPCs that drive properly and
# fire from cars (npcs/README.md). Optional: DiamondNav works with the stock one.
#
# open.mp ships NPC support as its own component, components/NPCs.so (NPCs.dll on
# Windows). This builds that same component from open.mp's source at OMP_TAG with
# the patches applied (patches/*.patch, in name order); put the result over the
# stock file. Only the component's insides change: the NPC natives live in the Pawn
# component and talk to it through the SDK interface, which the patches never
# touch, so the stock Pawn component keeps working.
#
#   Linux:    cmake -S npcs -B build/npcs -G Ninja -DCMAKE_BUILD_TYPE=Release \
#                 -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
#   Windows:  cmake -S npcs -B build/npcs -A Win32 -T ClangCL
#             cmake --build build/npcs --config Release
#
# The component must come from the same open.mp release as the server. When
# updating: move OMP_TAG and the submodule commits below to what the new tag pins,
# and re-check every patch still applies.

cmake_minimum_required(VERSION 3.19)
project(DiamondNavNPCs LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(OMP_TAG v1.5.8.3079)
set(OMP_VERSION_MAJOR 1)
set(OMP_VERSION_MINOR 5)
set(OMP_VERSION_PATCH 8)

find_package(Git REQUIRED)
include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# open.mp itself, only for the NPC component's sources. The patches are applied with
# `git apply` after a reset, so re-running CMake on a fetched tree is safe.
file(GLOB NPC_PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/patches/*.patch)
list(SORT NPC_PATCHES)
if(NPC_PATCHES)
    set(NPC_PATCH_COMMAND PATCH_COMMAND ${GIT_EXECUTABLE} checkout -- . && ${GIT_EXECUTABLE} apply --whitespace=nowarn ${NPC_PATCHES})
endif()
FetchContent_Declare(omp
    GIT_REPOSITORY https://github.com/openmultiplayer/open.mp
    GIT_TAG ${OMP_TAG}
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ""
    SOURCE_SUBDIR -
    ${NPC_PATCH_COMMAND})
# The submodules at the commits open.mp v1.5.8.3079 pins.
FetchContent_Declare(ompsdk
    GIT_REPOSITORY https://github.com/openmultiplayer/open.mp-sdk
    GIT_TAG efc11219c587dd1af791130bb937152051c05bda)
FetchContent_Declare(ompnetwork
    GIT_REPOSITORY https://github.com/openmultiplayer/open.mp-network
    GIT_TAG dc3eac9d5dc30f96edcf4e7e64f33d8c241d49ff
    SOURCE_SUBDIR -)
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/openmultiplayer/cpp-httplib
    GIT_TAG 12a865dad322c51bdfe3f78e2f5fee0e0ba8cb36
    SOURCE_SUBDIR -)
# open.mp gets this one from Conan; it is header-only.
FetchContent_Declare(ghcfs
    GIT_REPOSITORY https://github.com/gulrak/filesystem
    GIT_TAG v1.5.14
    SOURCE_SUBDIR -)
FetchContent_MakeAvailable(omp ompsdk ompnetwork httplib ghcfs)

# The two libraries open.mp's own tree builds from Shared/: the bitstream code, and the
# packet definitions on top of it (header-only).
file(GLOB_RECURSE NETWORK_SRC ${ompnetwork_SOURCE_DIR}/*.cpp)
add_library(omp_network STATIC ${NETWORK_SRC})
target_include_directories(omp_network PUBLIC ${ompnetwork_SOURCE_DIR})
target_link_libraries(omp_network PUBLIC OMP-SDK)

set(NPC_DIR ${omp_SOURCE_DIR}/Server/Components/NPCs)
file(GLOB_RECURSE NPC_SRC ${NPC_DIR}/*.cpp)
add_library(NPCs SHARED ${NPC_SRC})
set_target_properties(NPCs PROPERTIES PREFIX "" OUTPUT_NAME NPCs)
target_include_directories(NPCs PRIVATE
    ${omp_SOURCE_DIR}/Server                                   # <Server/Components/...> internals
    ${httplib_SOURCE_DIR}
    ${ghcfs_SOURCE_DIR}/include)
target_compile_definitions(NPCs PRIVATE
    OMP_VERSION_MAJOR=${OMP_VERSION_MAJOR}
    OMP_VERSION_MINOR=${OMP_VERSION_MINOR}
    OMP_VERSION_PATCH=${OMP_VERSION_PATCH})
# The packet definitions go last on the include path: NetCode has its own core.hpp,
# which must not shadow the SDK's.
add_library(omp_netcode INTERFACE)
target_include_directories(omp_netcode INTERFACE ${omp_SOURCE_DIR}/Shared/NetCode)
target_link_libraries(NPCs PRIVATE OMP-SDK omp_network omp_netcode)
if(UNIX)
    target_compile_options(NPCs PRIVATE -fvisibility=hidden -Wno-switch)
    # Ship one file: no libstdc++ version the VPS might not have.
    target_link_options(NPCs PRIVATE -static-libstdc++ -static-libgcc)
endif()
if(WIN32)
    target_compile_definitions(NPCs PRIVATE _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN _CRT_DECLARE_NONSTDC_NAMES _CRT_NONSTDC_NO_WARNINGS NOMINMAX)
    target_link_libraries(NPCs PRIVATE ws2_32)
endif()
if(MSVC)
    target_link_options(NPCs PRIVATE /LARGEADDRESSAWARE)
    set_property(TARGET NPCs omp_network PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
