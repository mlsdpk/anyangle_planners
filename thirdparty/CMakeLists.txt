set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(json)

set(tabulate_BUILD_TESTS OFF CACHE INTERNAL "") # Don't build the tests for tabulate.
set(SAMPLES OFF CACHE INTERNAL "") # Don't build the samples for tabulate.
add_subdirectory(tabulate)