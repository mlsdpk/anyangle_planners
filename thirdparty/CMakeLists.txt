set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(json)