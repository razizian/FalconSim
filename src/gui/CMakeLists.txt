# GUI module for FalconSim
if(FALCONSIM_BUILD_GUI)
    # Set up Qt resources - handle both Qt5 and Qt6
    if(QT_VERSION_MAJOR EQUAL 6)
        qt_add_resources(RESOURCES
            resources.qrc
        )
        
        # Qt automatic handling of UI files for Qt6
        qt_add_resources(UI_RESOURCES
            ui/MainWindow.ui
            ui/TelemetryWidget.ui
            ui/ControlPanel.ui
        )
    else() # Qt5
        qt5_add_resources(RESOURCES
            resources.qrc
        )
        
        # Qt automatic handling of UI files for Qt5
        qt5_wrap_ui(UI_RESOURCES
            ui/MainWindow.ui
            ui/TelemetryWidget.ui
            ui/ControlPanel.ui
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
        ${UI_RESOURCES}
    )

    # Set include directories
    target_include_directories(falconsim_gui PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Link with Qt and FalconSim library
    target_link_libraries(falconsim_gui PRIVATE
        falconsim
    )
    
    # Handle Qt version differences in linking
    if(QT_VERSION_MAJOR EQUAL 6)
        target_link_libraries(falconsim_gui PRIVATE
            Qt6::Core
            Qt6::Widgets
            Qt6::Quick
        )
    else() # Qt5
        target_link_libraries(falconsim_gui PRIVATE
            Qt5::Core
            Qt5::Widgets
            Qt5::Quick
        )
    endif()

    # Install rules
    install(TARGETS falconsim_gui
        RUNTIME DESTINATION bin
    )
endif() 