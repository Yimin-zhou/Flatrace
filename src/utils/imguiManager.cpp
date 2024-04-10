#include "imguiManager.h"
#include "globalState.h"

utils::ImGuiManager::ImGuiManager(SDL_Window* window, SDL_Renderer* renderer)
{
    // init imgui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void) io;
    io.IniFilename = nullptr;
    ImGui_ImplSDL2_InitForSDLRenderer(window);
    ImGui_ImplSDLRenderer_Init(renderer);
}

void utils::ImGuiManager::draw()
{
    ImGui_ImplSDLRenderer_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Render properties");
    if (!ImGui::IsWindowCollapsed())
    {
        ImGui::Text("Frame:");
        ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
        ImGui::SameLine();
        ImGui::Text("%.1f ms", 1000.0f / ImGui::GetIO().Framerate);
        ImGui::Separator();

        if (ImGui::Checkbox("OBB tracing", &GlobalState::enableOBB))
        {
//           GlobalState::heatmapView = false;
        }

        if (ImGui::Checkbox("Ray heatmap view", &GlobalState::heatmapView))
        {
            GlobalState::bboxView = false;
        }

        if (ImGui::Checkbox("BVH bounding box view", &GlobalState::bboxView))
        {
            GlobalState::heatmapView = false;
        }
    }
    ImGui::End();

    // draw imgui
    ImGui::Render();
    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
}
