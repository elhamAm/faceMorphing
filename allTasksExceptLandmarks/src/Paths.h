#ifndef __proj__Paths__
#define __proj__Paths__

#include <string>

using namespace std;
extern string path_faces_list;

// Paths to template data
extern string templateface;
extern string path_template;
extern string path_landmarks_template;

extern string path_template_for_rigidalign;
extern string path_landmarks_template_for_rigidalign;

// Paths to scan data
extern string path_landmarks_folder;
extern string path_cleanfaces_folder;
extern string path_rigidaligned_folder;
extern string path_nonrigidaligned_folder;

extern string demoface;
extern string path_landmarks_demofile;
extern string path_cleanfaces_demofile;
extern string path_rigidaligned_demofile;
extern string path_nonrigidaligned_demofile;

// For the landmark bonus task
extern string bonus_demoface;
extern string bonus_templateface;
extern string path_landmarks_bonus_demofile;
extern string path_landmarks_bonus_templatefile;
extern string path_cleanfaces_bonus_demofile;
extern string path_bonus_demofile_rigidaligned;
extern string path_bonus_template;
extern bool useBonusLandmarks;

#endif