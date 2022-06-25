include_directories (PRIVATE ".")

add_library (imgui
  imgui.cpp
  imgui_draw.cpp
  imgui_tables.cpp
  imgui_widgets.cpp
  imgui_impl_sdl.cpp
  imgui_impl_sdlrenderer.cpp
  imgui_demo.cpp)


