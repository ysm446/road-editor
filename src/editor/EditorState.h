#pragma once

enum class EditorMode
{
    Navigate,          // Camera only, no editing
    PolylineDraw,      // Click terrain to add road points
    PointEdit,         // Select / drag / delete existing road points
    IntersectionDraw,  // Click terrain to place intersections
    IntersectionEdit,  // Select / delete intersections
};
