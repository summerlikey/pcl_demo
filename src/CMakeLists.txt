# 递归获得目录下所有cpp文件
file(GLOB_RECURSE SRC_FILES ./*.cpp)
# 递归获得目录下所有的h文件
file(GLOB_RECURSE HEAD_FILES ./*.h ./*.hpp)

# process ui and moc action
# qt5_wrap_cpp(MOCCPP ${SRC_FILES})
# qt5_wrap_ui(WRAP_FILES ${UI_FILES})
# qt5_add_resources(RCC res.qrc) # ${RESOURCE_DIR}
add_library(pcl_demo_lib SHARED ${HEAD_FILES} ${SRC_FILES} )

add_executable(main main.cpp)
target_link_libraries(main ${INTERNAL_LIBS} ${THIRD_PARTY_LIBS} pcl_demo_lib)
