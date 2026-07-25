cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET "esp32p4")

# The camera entropy pipeline is shared with the master repo rather than copied,
# so both nodes run byte-identical extraction code. The two repos must sit as
# siblings on disk (D:\E-Lotto\elotto and D:\E-Lotto\elotto_slave).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../elotto/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(elotto_slave)
