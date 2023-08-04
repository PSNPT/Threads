function(add_example example_name example_sources)
  add_executable(${example_name} ${example_sources})

  target_link_libraries(
      ${example_name}
      PRIVATE
        external
  )

  # Require the C11 standard.
  set_target_properties(
      ${example_name}
      PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
  )

  target_compile_options(
      ${example_name}
      PRIVATE
        -Wall
  )
endfunction()

add_example(hot_potato hot_potato.c)
add_example(spin_and_join spin_and_join.c)
