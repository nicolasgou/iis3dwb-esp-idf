set(ROOT_PATH
    ${CMAKE_CURRENT_LIST_DIR}/..
)

set(COMPONENTS_PATH
    ${ROOT_PATH}/components
)

set(COMPONENT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}   "iis3dwb_self_test.c"
    ${COMPONENTS_PATH}/iis3dwb-pid "iis3dwb_reg.c"
)

set(COMPONENT_INCLUDE_DIRS
                                ${CMAKE_CURRENT_LIST_DIR}
                                ${COMPONENTS_PATH}/esp32_bus
                                ${COMPONENTS_PATH}/iis3dwb-pid
)

idf_component_register(SRCS ${COMPONENT_SOURCES}
                    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
)
