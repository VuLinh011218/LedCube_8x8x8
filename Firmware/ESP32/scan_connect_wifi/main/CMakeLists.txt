# Edit following two lines to set component requirements (see docs)
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES wifi
)
# # báo cho ESP-IDF biết có thêm thư viện ở thư mục lib
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib)

