add_subdirectory(third_party/gaden_gui)
install(
    TARGETS gaden_gui
    DESTINATION lib/${PROJECT_NAME}
)