#include "Paths.h"

using namespace std;

string path_faces_list = "../data/faces_list";

// Paths to template data
string templateface = "template_mid";
string path_template =  "../data/" + templateface + ".obj";
string path_landmarks_template = "../data/" + templateface + "_landmarks";

string path_template_for_rigidalign =  "../data/template_rigid_aligned.obj";
string path_landmarks_template_for_rigidalign = "../data/template_landmarks";


// Paths to scan data
string path_landmarks_folder = "../data/landmarks/";
string path_cleanfaces_folder = "../data/scanned_faces_cleaned_consistent/";
string path_rigidaligned_folder = "../data/rigidAlignedFaces/";
string path_nonrigidaligned_folder = "../data/nonRigidFaces/";

string demoface = "alain_normal";
string path_landmarks_demofile = "../data/landmarks/" + demoface + "_landmarks";
string path_cleanfaces_demofile = "../data/scanned_faces_cleaned_consistent/" + demoface + ".obj";
string path_rigidaligned_demofile = "../data/rigidAlignedFaces/" + demoface + "_rigid_aligned.obj";
string path_nonrigidaligned_demofile = "../data/nonRigidFaces/" + demoface + "_non_rigid_aligned.obj";

// For the landmark bonus task
bool useBonusLandmarks = false;
string bonus_demoface = "alain_normal";
string bonus_templateface = "template_high";
string path_landmarks_bonus_demofile = "../data/landmarksDL/" + bonus_demoface + "_landmarks.txt";
string path_landmarks_bonus_templatefile = "../data/landmarksDL/" + bonus_templateface + "_landmarks";
string path_cleanfaces_bonus_demofile = "../data/scanned_faces_cleaned_consistent/" + bonus_demoface + ".obj";
string path_bonus_demofile_rigidaligned = "../data/rigidAlignedFaces/" + bonus_demoface + "_rigid_aligned.obj";
string path_bonus_template = "../data/" + bonus_templateface + ".obj";
