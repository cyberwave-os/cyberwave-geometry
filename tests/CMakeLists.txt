# Test-only support code. The JSON codec lives here rather than in the library
# so the core keeps its promise of building against nothing but a standard
# library.
add_library(cyberwave_geometry_testing STATIC
    test_support.cpp
    mini_json.cpp
    golden_ops.cpp
)
target_include_directories(cyberwave_geometry_testing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cyberwave_geometry_testing PUBLIC cyberwave_geometry)
if(NOT MSVC)
    target_compile_options(cyberwave_geometry_testing PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

function(cyberwave_geometry_add_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE cyberwave_geometry_testing)
    if(NOT MSVC)
        target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

cyberwave_geometry_add_test(test_quaternion)
cyberwave_geometry_add_test(test_transform)
cyberwave_geometry_add_test(test_fk)
cyberwave_geometry_add_test(test_geodetic)

if(CYBERWAVE_GEOMETRY_BUILD_C_API)
    cyberwave_geometry_add_test(test_c_api)
    target_link_libraries(test_c_api PRIVATE cyberwave_geometry_c)
endif()

# Golden vectors: the same file every language binding's conformance runner
# reads. `--check` re-evaluates each case and compares numerically, so a
# platform whose libm differs in the last ulp still passes while a genuine
# behaviour change fails.
add_executable(golden_gen golden_gen.cpp)
target_link_libraries(golden_gen PRIVATE cyberwave_geometry_testing)
if(NOT MSVC)
    target_compile_options(golden_gen PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

set(CYBERWAVE_GEOMETRY_GOLDEN_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../golden/geometry_golden.json")
add_test(NAME golden_vectors COMMAND golden_gen --check "${CYBERWAVE_GEOMETRY_GOLDEN_FILE}")
