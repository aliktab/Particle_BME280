
cmake_minimum_required(VERSION 3.3)

project(SmartPower)


# service functions

function (add_target target_name)

  add_executable(${target_name} src/${target_name}.cpp)

  set_target_properties(${target_name} PROPERTIES LINKER_LANGUAGE C)

endfunction (add_target)


function (add_lib lib_name)

  add_library(${lib_name} src/${lib_name}.cpp)

endfunction (add_lib)


function (add_example example_name)

  add_library(${example_name} examples/${example_name}/${example_name}.ino)

  set_target_properties(${example_name} PROPERTIES LINKER_LANGUAGE C)

endfunction (add_example)


# common for particle

include_directories(
    ../firmware/system/inc
    ../firmware/hal/inc
    ../firmware/hal/shared
    ../firmware/hal/src/stm32f2xx
    ../firmware/services/inc
    ../firmware/wiring/inc
    ../firmware/user/inc
    ../powershield/src
    ../Adafruit/SSD1306/src
  )


# add targets

add_target(BME280)

add_example(usage)




