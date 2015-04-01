rock_add_ruby_package(transformer
    RUBY_FILES lib/transformer.rb lib/transformer)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/syskit_plugin.rb
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/orogen/plugins/syskit
        RENAME transformer_plugin.rb)
