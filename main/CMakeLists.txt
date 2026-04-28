add_executable(zhinengche main.cpp)

target_link_libraries(zhinengche
    PRIVATE
        Threads::Threads
        common_lib
        ${OpenCV_LIBS}
)
