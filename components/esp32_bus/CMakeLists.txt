# Copyright (c) IASX Tecnologia S/A. All rights reserved.
# SPDX-License-Identifier: MIT

set(ROOT_PATH
    ${CMAKE_CURRENT_LIST_DIR}/../..
)

set(COMPONENTS_PATH
    ${ROOT_PATH}/components
)




set(COMPONENT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}   "esp32_bus.c"
)



set(COMPONENT_INCLUDE_DIRS
                                ${CMAKE_CURRENT_LIST_DIR}

)






idf_component_register(SRCS ${COMPONENT_SOURCES}
                    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
)




