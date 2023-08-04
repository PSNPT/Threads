set(THREAD_LIB ${PROJECT_NAME})

add_library(
  ${THREAD_LIB}
  interrupts.h
  interrupts.c
  thread.h
  thread.c
)

add_library(external ALIAS ${THREAD_LIB})

target_include_directories(
  ${THREAD_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Require the C11 standard.
set_target_properties(
  ${THREAD_LIB}
  PROPERTIES
      C_STANDARD 11
      C_STANDARD_REQUIRED ON
)

target_compile_options(
    ${THREAD_LIB}
    PRIVATE
      -D_GNU_SOURCE
)
