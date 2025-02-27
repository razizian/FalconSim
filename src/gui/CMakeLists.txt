# GUI module for FalconSim
if(FALCONSIM_BUILD_GUI)
    # Set up Qt resources - handle both Qt5 and Qt6
    if(QT_VERSION_MAJOR EQUAL 6)
        # Find additional Qt modules
        find_package(Qt6 COMPONENTS OpenGLWidgets REQUIRED)
        
        # Set up automatic MOC, UIC, and RCC processing
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_AUTOUIC ON)
        set(CMAKE_AUTORCC ON)
        
        # Set the UI file include directory
        set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
        
        # Add resources
        qt_add_resources(RESOURCES
            resources.qrc
        )
    else() # Qt5
        # Find additional Qt modules
        find_package(Qt5 COMPONENTS OpenGL REQUIRED)
        
        # Set up automatic MOC, UIC, and RCC processing
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_AUTOUIC ON)
        set(CMAKE_AUTORCC ON)
        
        # Set the UI file include directory
        set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
        
        # Add resources
        qt5_add_resources(RESOURCES
            resources.qrc
        )
    endif()

    # Create GUI executable
    add_executable(falconsim_gui
        main.cpp
        MainWindow.cpp
        TelemetryWidget.cpp
        ControlPanel.cpp
        Flight3DView.cpp
        ${RESOURCES}
    )

    # Set include directories
    target_include_directories(falconsim_gui PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Link with Qt and FalconSim library
    target_link_libraries(falconsim_gui PRIVATE
        falconsim
        ${OPENGL_LIBRARIES}
    )
    
    # Handle Qt version differences in linking
    if(QT_VERSION_MAJOR EQUAL 6)
        target_link_libraries(falconsim_gui PRIVATE
            Qt6::Core
            Qt6::Widgets
            Qt6::Quick
            Qt6::OpenGLWidgets
        )
    else() # Qt5
        target_link_libraries(falconsim_gui PRIVATE
            Qt5::Core
            Qt5::Widgets
            Qt5::Quick
            Qt5::OpenGL
        )
    endif()

    # Install rules
    install(TARGETS falconsim_gui
        RUNTIME DESTINATION bin
    )
endif() 