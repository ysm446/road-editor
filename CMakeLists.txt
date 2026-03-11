cmake_minimum_required(VERSION 3.20)
project(RoadEditor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ── ImGui (docking branch) ──────────────────────────────────────────────────
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# ── nlohmann/json ───────────────────────────────────────────────────────────
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(json)

# ── stb ─────────────────────────────────────────────────────────────────────
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(stb)

# ── Source files ─────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/App.cpp
    src/renderer/D3D11Context.cpp
    src/renderer/Shader.cpp
    src/scene/Camera.cpp
    src/scene/Grid.cpp
    src/scene/Terrain.cpp
    src/renderer/DebugDraw.cpp
    src/editor/RoadData.cpp
    src/editor/PolylineEditor.cpp
    src/ui/ImGuiLayer.cpp
    # ImGui
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
)

add_executable(RoadEditor WIN32 ${SOURCES})

target_include_directories(RoadEditor PRIVATE
    src
    src/editor
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${stb_SOURCE_DIR}
)

target_link_libraries(RoadEditor PRIVATE
    d3d11
    dxgi
    d3dcompiler
    comdlg32
    nlohmann_json::nlohmann_json
)

target_compile_definitions(RoadEditor PRIVATE
    $<$<CONFIG:Debug>:_DEBUG>
    UNICODE
    _UNICODE
)

# Copy shaders to output directory after build
add_custom_command(TARGET RoadEditor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:RoadEditor>/shaders
    COMMENT "Copying shaders..."
)
