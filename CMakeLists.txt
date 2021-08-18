cmake_minimum_required (VERSION 3.5)

project(AttitudeMathLib)

# Add sub directories
add_subdirectory(AML)
add_subdirectory(example)
add_subdirectory(test)

enable_testing()
add_test(test_all test/AML_Test)
