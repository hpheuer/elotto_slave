cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET "esp32p4")

# The camera entropy pipeline (elotto_camera), the UDP wire format (elotto_link)
# and the update/boot-safety logic (elotto_ota) are shared with the master repo
# rather than copied, so both nodes run byte-identical extraction code, agree on
# the framing by construction, and cannot disagree about what makes a node
# reachable. The two repos must sit as siblings on disk
# (D:\E-Lotto\elotto and D:\E-Lotto\elotto_slave).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../elotto/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(elotto_slave)
