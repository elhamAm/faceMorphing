#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/slice_into.h>
#include <igl/rotate_by_quat.h>


#include <igl/readOFF.h>
#include <igl/writeDMAT.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/ViewerData.h>
#include <iostream>

#include "Lasso.h"
#include "Colors.h"

//activate this for alternate UI (easier to debug)
//#define UPDATE_ONLY_ON_UP

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

Viewer viewer;

//vertex array, #V x3
Eigen::MatrixXd V(0, 3);
//face array, #F x3
Eigen::MatrixXi F(0, 3);

//mouse interaction
enum MouseMode
{
    SELECT,
    TRANSLATE,
    ROTATE,
    NONE
};
MouseMode mouse_mode = NONE;
bool doit = false;
int down_mouse_x = -1, down_mouse_y = -1;

//per vertex color array, #V x3
Eigen::MatrixXd vertex_colors;
int current_selected_ind = -1;
vector<int> landmarks_ind;
string landmarks_filename;


//function declarations (see below for implementation)
bool solve(Viewer &viewer);
bool callback_mouse_down(Viewer &viewer, int button, int modifier);
bool callback_pre_draw(Viewer &viewer);
bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers);
void applySelection();
void exportLandmarks(string filename);
void computeNamingConventions(string filename);



void computeNamingConventions(string filename) {

    size_t lastindex = filename.find_last_of(".");
    string rawname = filename.substr(0, lastindex);

    landmarks_filename = rawname.append("_landmarks");
}


bool load_mesh(string filename)
{

    computeNamingConventions(filename);

    igl::readOBJ(filename, V, F);
    viewer.data().clear();
    viewer.data().set_mesh(V, F);

    viewer.core.align_camera_center(V);

    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "Usage assignment6 mesh.off>" << endl;
        load_mesh("../../../data/headtemplate.obj");
    }
    else
    {
        load_mesh(argv[1]);
    }

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]() {
        // Draw parent menu content
        menu.draw_viewer_menu();

        // Add new group
        if (ImGui::CollapsingHeader("Deformation Controls", ImGuiTreeNodeFlags_DefaultOpen))
        {
            int mouse_mode_type = static_cast<int>(mouse_mode);

            if (ImGui::Combo("Mouse Mode", &mouse_mode_type, "SELECT\0TRANSLATE\0ROTATE\0NONE\0"))
            {
                mouse_mode = static_cast<MouseMode>(mouse_mode_type);
            }

            if (ImGui::Button("Clear Selection", ImVec2(-1, 0)))
            {
                landmarks_ind.clear();
                current_selected_ind = -1;
                viewer.data().clear_labels();
            }

            if (ImGui::Button("Apply Selection", ImVec2(-1, 0)))
            {
                applySelection();
            }

            if (ImGui::Button("Export landmarks", ImVec2(-1, 0)))
            {
                exportLandmarks(landmarks_filename);
            }
        }
    };

    viewer.callback_key_down = callback_key_down;
    viewer.callback_mouse_down = callback_mouse_down;
    viewer.callback_pre_draw = callback_pre_draw;

    viewer.data().point_size = 10;
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
}

bool callback_mouse_down(Viewer &viewer, int button, int modifier)
{
    if (button == (int)Viewer::MouseButton::Right)
        return false;

    down_mouse_x = viewer.current_mouse_x;
    down_mouse_y = viewer.current_mouse_y;

    if (mouse_mode == SELECT)
    {
        int fid;
        Eigen::Vector3f bc;
        // Cast a ray in the view direction starting from the mouse position
        double x = down_mouse_x;
        double y = viewer.core.viewport(3) - down_mouse_y;
        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view,
            viewer.core.proj, viewer.core.viewport, V, F, fid, bc))
        {
            // paint hit red
            int mostImportantVertex = bc.maxCoeff(); // nearest
            int cid = F.row(fid)[mostImportantVertex];

            current_selected_ind = cid;
            //vertex_colors.row(cid) << 1, 0, 0;
            
            viewer.data().set_colors(vertex_colors);
            return true;
        }
    }
    return doit;
}

bool callback_pre_draw(Viewer &viewer)
{
    // initialize vertex colors
    vertex_colors = Eigen::MatrixXd::Constant(V.rows(), 3, 0.9);

    // first, color constraints
    for (int i = 0; i < landmarks_ind.size(); ++i) {
        int v_i = landmarks_ind[i];
        vertex_colors.row(v_i) << regionColors[1][0], regionColors[1][1], regionColors[1][2];
        viewer.data().add_label(V.row(v_i), to_string(i+1));
    }

    // then, color selection
    if (current_selected_ind >= 0)
    {
        vertex_colors.row(current_selected_ind) << 131. / 255, 131. / 255, 131. / 255.;
    }

    viewer.data().set_colors(vertex_colors);
    viewer.data().V_material_specular.fill(0);
    viewer.data().V_material_specular.col(3).fill(1);
    viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_DIFFUSE | igl::opengl::MeshGL::DIRTY_SPECULAR;

    //clear points and lines
    viewer.data().set_points(Eigen::MatrixXd::Zero(0, 3), Eigen::MatrixXd::Zero(0, 3));
    viewer.data().set_edges(Eigen::MatrixXd::Zero(0, 3), Eigen::MatrixXi::Zero(0, 3), Eigen::MatrixXd::Zero(0, 3));

    // update the vertex position all the time
    viewer.data().V.resize(V.rows(), 3);
    viewer.data().V << V;

    viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_POSITION;
    return false;
}

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers)
{
    bool handled = false;
    if (key == 'S')
    {
        mouse_mode = SELECT;
        handled = true;
    }
    if (key == 'A')
    {
        applySelection();
        callback_key_down(viewer, '1', 0);
        handled = true;
    }
    return handled;
}

void applySelection()
{
    if (current_selected_ind >= 0)
    {
        landmarks_ind.push_back(current_selected_ind);
        current_selected_ind = -1;
    }
}

void exportLandmarks(string filename) {
    int num_of_landmarks = landmarks_ind.size();
    if (num_of_landmarks <= 0)
    {
        return;
    }

    MatrixXi landmarks(num_of_landmarks, 2);
    ofstream outfile(filename);
    for (int i = 0; i < landmarks_ind.size(); ++i) {
        int v_i = landmarks_ind[i];
        int p_i = i + 1;
        outfile << v_i << " " << p_i << endl;
    }
    outfile.close();
}
