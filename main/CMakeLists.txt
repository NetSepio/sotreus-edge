idf_component_register(
    SRCS
        "main.c"
        "edge_api.c"
        "edge_battery.c"
        "edge_ble.c"
        "edge_config.c"
        "edge_console.c"
        "edge_dns.c"
        "edge_events.c"
        "edge_http.c"
        "edge_log.c"
        "edge_portal.c"
        "edge_records.c"
        "edge_time.c"
        "edge_util.c"
        "edge_wifi.c"
        "sense.c"
        "ssd1306.c"
        "ui.c"
        "mesh/mesh.c"
        "mesh/mesh_pb.c" "mesh/mesh_crypto.c" "mesh/mesh_pb_ext.c" "mesh/mesh_admin.c" "mesh/mesh_sfdb.c" "mesh/mesh_mqtt.c"
        "mesh/mesh_radio.c"
        "mesh/sx1262.c"
    INCLUDE_DIRS "." "mesh"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)

# Web assets: text files are gzipped at build time (served with Content-Encoding: gzip),
# fonts are already compressed. Each file is embedded as _binary_<name>_start/_end.
idf_build_get_property(python PYTHON)
set(WEB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/web")
set(WEB_GZIP
    "index.html"
    "vendor/maplibre-gl-4.7.1.js"
    "vendor/maplibre-gl-4.7.1.css"
    "vendor/uplot-1.6.31.min.js"
    "vendor/uplot-1.6.31.min.css"
    "vendor/satellite-5.0.0.min.js"
)
foreach(rel ${WEB_GZIP})
    get_filename_component(name "${rel}" NAME)
    set(out "${CMAKE_CURRENT_BINARY_DIR}/web/${name}.gz")
    add_custom_command(
        OUTPUT "${out}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/web"
        COMMAND ${python} -c "import gzip,sys; d=open(sys.argv[1],'rb').read(); open(sys.argv[2],'wb').write(gzip.compress(d,9,mtime=0))" "${WEB_DIR}/${rel}" "${out}"
        DEPENDS "${WEB_DIR}/${rel}"
        VERBATIM)
    target_add_binary_data(${COMPONENT_TARGET} "${out}" BINARY DEPENDS "${out}")
endforeach()
foreach(font geist_regular geist_medium geist_mono_regular instrument_serif_regular)
    target_add_binary_data(${COMPONENT_TARGET} "${WEB_DIR}/fonts/${font}.woff2" BINARY)
endforeach()
