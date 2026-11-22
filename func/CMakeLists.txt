add_library(func_uart STATIC)

target_sources(func_uart PRIVATE
    # ${CMAKE_CURRENT_SOURCE_DIR}/uart/func_uart.c   # 与 func_uart_0.c 同名符号冲突，暂不参与构建
    ${CMAKE_CURRENT_SOURCE_DIR}/uart/circle_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/uart/func_uart_0.c
)

target_include_directories(func_uart
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/uart
)

target_link_libraries(func_uart PUBLIC stm32cubemx)

