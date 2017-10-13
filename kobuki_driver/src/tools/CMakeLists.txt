###############################################################################
# Tools
###############################################################################

add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki)
install(TARGETS version_info
        DESTINATION bin
)

if (NOT ${MSVC})
add_executable(simple_keyop simple_keyop.cpp)
target_link_libraries(simple_keyop kobuki)
install(TARGETS simple_keyop
        DESTINATION bin
)
endif()
