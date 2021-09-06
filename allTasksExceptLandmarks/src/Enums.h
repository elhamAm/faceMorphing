#ifndef __proj__Enums__
#define __proj__Enums__

enum BoundaryMode { CLOSEST_BOUNDARY_VERTEX, CLOSEST_VERTEX, FIX_TEMPLATE_BOUNDARY, NONE };

enum ConstraintsMode { ALL, LANDMARK_BOUNDARY, UPDATE_CLOSEPOINTS};

enum VisualizationMode { WARP, WARP_WITH_CONSTRAINTS, TEMPLATE, SCAN, TEMPLATE_WITH_OVERLAY, TEMPLATE_WITH_CONSTRAINTS, TEMPLATE_WITH_WARPED};

//mouse interaction
enum MouseMode
{
    SELECT,
    TRANSLATE,
    ROTATE,
    NOTHING
};
#endif