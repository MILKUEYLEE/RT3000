cmake_minimum_required(VERSION 3.0)

project(iVH_Interface_Rt_3000 VERSION 0.1.0 LANGUAGES CXX)

find_package(Boost 1.63 REQUIRED thread chrono filesystem)

set(VTD_DEVELOP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "Path to VTD Develop directory")

add_library(${PROJECT_NAME} SHARED
    inc/iVH_Interface_Rt_3000.hh
    src/iVH_Interface_Rt_3000.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ./inc
    ${VTD_DEVELOP_ROOT}/Modules/Common/inc
    ${VTD_DEVELOP_ROOT}/Framework/inc
    ${VTD_DEVELOP_ROOT}/Communication/Common
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE  # SYSTEM: warnings turned off for these headers
    ./OpenDRIVE/inc
    ./OpenDRIVE/inc/Nodes
    ./OpenDRIVE/Public/inc
    ./OpenDRIVE/Public/inc/BaseNodes
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    $<$<CONFIG:Debug>:VTD_DEBUG>
    $<$<CONFIG:Debug>:_DEBUG>
    LINUX
    VTD_LIB_REVISION="4.4.0" #in future versions this define will be propagated by the VTD framework
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    DEBUG_POSTFIX "-d"
    VERSION ${PROJECT_VERSION}
)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    option(USE_CXX11_ABI OFF "See GCC's libstdc++ Dual-ABI")
    target_compile_definitions(${PROJECT_NAME} PUBLIC _GLIBCXX_USE_CXX11_ABI=$<BOOL:${USE_CXX11_ABI}>)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -fsigned-char -Wconversion -pedantic)
endif()

set(mm_dir_postfix "cxx98") 
if(USE_CXX11_ABI)
    set(mm_dir_postfix "cxx11")
endif()

add_library(VTDModulePlugin SHARED IMPORTED)
set_target_properties(VTDModulePlugin PROPERTIES
    IMPORTED_LOCATION ${VTD_DEVELOP_ROOT}/../Runtime/Core/ModuleManager.${mm_dir_postfix}/lib/libVTDModulePlugin.so
)

target_link_libraries(${PROJECT_NAME} PRIVATE dl VTDModulePlugin ${CMAKE_CURRENT_SOURCE_DIR}/OpenDRIVE/lib/libODrive.1.5.5.a Boost::thread Boost::chrono Boost::system)

set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
include(CPack)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ./ CONFIGURATIONS)
install(FILES ReleaseNotes.txt DESTINATION ./ OPTIONAL)


