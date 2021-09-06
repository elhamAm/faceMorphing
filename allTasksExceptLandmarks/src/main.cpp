#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

#include "NonRigid.h"
#include "Enums.h"
#include "DecimateTemplate.h"
#include "RigidAlignment.h"
#include "PCA.h"

#include "Paths.h"

using namespace std;
using namespace Eigen;

using Viewer = igl::opengl::glfw::Viewer;

typedef Triplet<double> T;

Viewer viewer;


/*------------------------------------------------------
                Variables for Task 3
-------------------------------------------------------*/
RigidAlignment rigid;

// Gui
int mesh_type = 1;
string filename;

/*------------------------------------------------------
                Variables for Task 4
-------------------------------------------------------*/
NonRigid nonRigid;

//Gui
BoundaryMode boundary_mode = FIX_TEMPLATE_BOUNDARY;
VisualizationMode visualization_mode = WARP;
ConstraintsMode constraints_mode = LANDMARK_BOUNDARY;
double global_distance_thresh = 0.05; // distance threshhold for closepoint_constraints
double lambda = 0.5;
int iterations = 1;

/*------------------------------------------------------
                Variables for Task 5
-------------------------------------------------------*/
PCA pca;

//Gui
std::vector<float> eigenfaceStrength;
bool visualizeEigenfaces = false;
// Reconstruction
bool reconstructionAnalysis = false;
int maxEigenfaces = 10;

// Morphing
bool visualizeMorphing = false;
float morphing = 0.0;
int id1 = 0;
int id2 = 1;

bool callback_key_pressed(Viewer &viewer, unsigned char key, int modifiers)
{
    string resultName = "../results/Newperson.obj";
    int numIds = 1;
    int staticIterations = 1;
    int dynmaicIterations = 1;
    double lambda_all = 0.5;

    switch (key)
    {
    case '1':
        break;
    case '2':
        nonRigid.nonRigidAlignment(resultName, lambda, iterations, constraints_mode, global_distance_thresh);
        nonRigid.visualizeResult(viewer, visualization_mode);
        break;
    case '3':
        nonRigid.nonRigidAlignmentForAll(staticIterations, dynmaicIterations, lambda_all, iterations, constraints_mode, global_distance_thresh);
        break;
    case '4':
        pca.facePCA();
        break;
    }
    return true;
}


int main(int argc, char *argv[])
{
    nonRigid = NonRigid();
    string resultName = "../results/Newperson.obj";
    int numIds = 1;
    int staticIterations = 2;
    int dynmaicIterations = 20;
    double lambda_all = 0.5;

    /* We used this code to create the decimated template meshes:

	Eigen::MatrixXd V_template;
	Eigen::MatrixXi F_template;
	string path_to_template = "../data/template_rigid_aligned.obj";
	igl::read_triangle_mesh(path_to_template, V_template, F_template);
	DecimateTemplate dt = DecimateTemplate(V_template, F_template);
	dt.createDecimations();
	dt.visualize(viewer);
	*/

    cout << "Demo Face: " << demoface << endl;
    rigid = RigidAlignment();
    rigid.loadData();
    mesh_type = 1;
    rigid.showMesh(viewer, mesh_type);
    rigid.drawLandmarks(viewer);

    
    pca = PCA();
    pca.prepare(viewer);
    eigenfaceStrength.resize(10);


    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]()
    {
        // Add new group
        menu.draw_viewer_menu();

        // Add new group
        if (ImGui::CollapsingHeader("Task 3: Rigid Alignment", ImGuiTreeNodeFlags_CollapsingHeader))
        {
            ImGui::Checkbox("useBonusLandmarks", &useBonusLandmarks);
            if (ImGui::Button("Load Data", ImVec2(-1, 0)))
            {
                rigid.loadData();
                mesh_type = 1;
                rigid.showMesh(viewer, mesh_type);
                rigid.drawLandmarks(viewer);
            }
            if (ImGui::Button("Perform rigid alignment", ImVec2(-1, 0)))
            {
                printf("Perform rigid alignment\n");
                rigid.rigidAlignment(true);
                rigid.fillLandmarkVertexPositions(true);
                rigid.showMesh(viewer, mesh_type);
                rigid.drawLandmarks(viewer);
                if(!useBonusLandmarks){
                    rigid.saveMesh(filename, true);
                }
            }

            if (ImGui::Combo("Mesh type", &mesh_type, "Scan\0Template\0Scan & Template Overlaid\0"))
            {
                std::cout << "mesh_type: " << mesh_type << std::endl;

                rigid.showMesh(viewer, mesh_type);
                rigid.drawLandmarks(viewer);
            }

            ImGui::InputText("File name", filename);
            if (ImGui::Button("Align All", ImVec2(-1, 0)))
            {
                rigid.rigidAlignmentAll(true);

                rigid.showMesh(viewer, false);
            }
        }

        if (ImGui::CollapsingHeader("Task 4: Non-Rigid Alignment"))
        {
            if (ImGui::Button("Load Data ", ImVec2(-1, 0)))
            {
                cout << "LoadData" << endl; 
                nonRigid.loadData();
                nonRigid.visualizeResult(viewer, visualization_mode);
            }
            if (ImGui::Button("Iteration Demo", ImVec2(-1, 0)))
            {
                nonRigid.nonRigidAlignment(resultName, lambda, iterations, constraints_mode, global_distance_thresh);
                nonRigid.visualizeResult(viewer, visualization_mode);
            }
            if (ImGui::Button("Compute All", ImVec2(-1, 0)))
            {
                nonRigid.nonRigidAlignmentForAll(staticIterations, dynmaicIterations, lambda_all, iterations, constraints_mode, global_distance_thresh);
            }
            ImGui::InputDouble("Close Points Threshold", &global_distance_thresh);
            ImGui::InputDouble("Lambda", &lambda);
            ImGui::InputInt("Iterations", &iterations);

            int visualization_mode_type = static_cast<int>(visualization_mode);
            if (ImGui::Combo("Visualization Mode", &visualization_mode_type, "WARP\0WARP_WITH_CONSTRAINTS\0TEMPLATE\0SCAN\0TEMPLATE_WITH_OVERLAY\0TEMPLATE_WITH_CONSTRAINTS\0TEMPLATE_WITH_WARPED\0"))
            {
                visualization_mode = static_cast<VisualizationMode>(visualization_mode_type);
                nonRigid.visualizeResult(viewer, visualization_mode);
            }
            int constraints_mode_type = static_cast<int>(constraints_mode);
            if (ImGui::Combo("Constraints Mode", &constraints_mode_type, "ALL\0LANDMARK_BOUNDARY\0UPDATE_CLOSEPOINTS\0"))
            {
                constraints_mode = static_cast<ConstraintsMode>(constraints_mode_type);
            }
        }
        if (ImGui::CollapsingHeader("Task 5: Eigen Vector Visualization"))
        {
            if (ImGui::Button("Perform PCA", ImVec2(-1, 0)))
            {
                printf("Perform PCA\n");

                pca.facePCA();
            }
            ImGui::Checkbox("Visualize Eigenfaces Dynamic", &visualizeEigenfaces);
            ImGui::SliderFloat("EV1", &eigenfaceStrength[0], -100.0, 100.0);
            ImGui::SliderFloat("EV2", &eigenfaceStrength[1], -100.0, 100.0);
            ImGui::SliderFloat("EV3", &eigenfaceStrength[2], -100.0, 100.0);
            ImGui::SliderFloat("EV4", &eigenfaceStrength[3], -100.0, 100.0);
            ImGui::SliderFloat("EV5", &eigenfaceStrength[4], -100.0, 100.0);
            ImGui::SliderFloat("EV6", &eigenfaceStrength[5], -100.0, 100.0);
            ImGui::SliderFloat("EV7", &eigenfaceStrength[6], -100.0, 100.0);
            ImGui::SliderFloat("EV8", &eigenfaceStrength[7], -100.0, 100.0);
            ImGui::SliderFloat("EV9", &eigenfaceStrength[8], -100.0, 100.0);
            ImGui::SliderFloat("EV10", &eigenfaceStrength[9], -100.0, 100.0);
            if (visualizeEigenfaces)
            {
                eigenfaceStrength.resize(10);
                pca.show_eigen_face(viewer, eigenfaceStrength);
            }
            if (ImGui::Button("Show Mean Face", ImVec2(-1, 0)))
            {
                visualizeEigenfaces = false;
                pca.show_mean_face(viewer);
            }
            if (ImGui::Button("Show Loaded Face", ImVec2(-1, 0)))
            {
                visualizeEigenfaces = false;
                pca.show_face(viewer);
            }
            if (ImGui::Button("Project Loaded Mesh", ImVec2(-1, 0)))
            {
                eigenfaceStrength = pca.project_face(pca.V);
            }
        }
        if (ImGui::CollapsingHeader("Task 5: Reconstruction Analysis"))
        {
            ImGui::Checkbox("Activate Reconstruction Analysis", &reconstructionAnalysis);
            ImGui::InputInt("Eigenfaces", &maxEigenfaces, 1, 1);
            if (reconstructionAnalysis)
            {
                visualizeEigenfaces = false;
                maxEigenfaces = pca.reconstruction_analysis(viewer, maxEigenfaces);
            }
            double error = pca.mesh_difference();
            ImGui::InputDouble("Error: ", &error);
        }
        if (ImGui::CollapsingHeader("Task 5: Morphing"))
        {
            ImGui::Checkbox("Morph faces", &visualizeMorphing);
            ImGui::InputInt("first ID", &id1, 1, 1);
            ImGui::InputInt("second ID", &id2, 1, 1);
            ImGui::SliderFloat("Morphing", &morphing, 0.0, 1.0);
            if (visualizeMorphing)
            {
                visualizeEigenfaces = false;
                pca.morph_faces(viewer, morphing, id1, id2);
            }
        }
    };

    viewer.callback_key_pressed = callback_key_pressed;

    viewer.launch();
}
