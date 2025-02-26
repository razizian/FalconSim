add_subdirectory(core)
# Will add physics when implemented
add_subdirectory(physics)
add_subdirectory(network)

if(FALCONSIM_BUILD_GUI)
    add_subdirectory(gui)
endif() 