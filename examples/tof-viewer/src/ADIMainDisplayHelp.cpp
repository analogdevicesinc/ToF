#include "ADIMainWindow.h"
#include "imgui_md.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace adiMainWindow;

ImFont* g_font_regular;
ImFont* g_font_bold;
ImFont* g_font_bold_large;

struct help_markdown : public imgui_md
{
	ImFont* get_font() const override
	{
		if (m_is_table_header) {
			return g_font_bold;
		}

		switch (m_hlevel)
		{
		case 0:
			return m_is_strong ? g_font_bold : g_font_regular;
		case 1:
			return g_font_bold_large;
		default:
			return g_font_bold;
		}
	};

	void open_url() const override
	{
		//platform dependent code
		//SDL_OpenURL(m_href.c_str());
	}

	bool get_image(image_info& nfo) const override
	{
		//use m_href to identify images
		//nfo.texture_id = g_texture1;
		nfo.size = { 40,20 };
		nfo.uv0 = { 0,0 };
		nfo.uv1 = { 1,1 };
		nfo.col_tint = { 1,1,1,1 };
		nfo.col_border = { 0,0,0,0 };
		return true;
	}

	void html_div(const std::string& dclass, bool e) override
	{
		if (dclass == "red") {
			if (e) {
				m_table_border = false;
				ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
			}
			else {
				ImGui::PopStyleColor();
				m_table_border = true;
			}
		}
	}
};

void ADIMainWindow::DisplayHelp() {

	static std::string help_content = "";

	centreWindow(1000.0f * m_dpi_scale_factor,1000.0f * m_dpi_scale_factor);

	if (ImGui::BeginPopupModal("Help Window", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {

		if (help_content.empty()) {
			std::ifstream help_file("tof-viewer.md");
			if (help_file) {
				LOG(INFO) << "Loading help content from file.";
				std::string line;
				while (std::getline(help_file, line)) {
					help_content += line + '\n';
				}
			}
			else {
				LOG(ERROR) << "Failed to open help file.";
				help_content = "Help content could not be loaded.";
			}
		}

		if (ImGui::Button("Close"))
			ImGui::CloseCurrentPopup();

		static help_markdown s_printer;
		s_printer.print(help_content.c_str(), help_content.c_str() + help_content.size());

		ImGui::EndPopup();
	}
}