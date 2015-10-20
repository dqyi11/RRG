set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(rrg-viz-demo
               configobjdialog.h
               configobjdialog.cpp
               mainwindow.h
               mainwindow.cpp
               rrg_viz_demo.cpp
               )

include_directories(
                   )

target_link_libraries(rrg-viz-demo
                      rrg
                      rrg-viz
                      ${QT_LIBRARIES}
                     )
