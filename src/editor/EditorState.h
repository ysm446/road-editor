#pragma once

enum class EditorMode
{
    Navigate,          // Camera only, no editing
    PolylineDraw,      // Click terrain to add road points
    PointEdit,         // Select / drag / delete existing road points
    VerticalCurveEdit, // Placeholder for vertical curve editing
    BankAngleEdit,     // Placeholder for bank angle editing
    IntersectionDraw,  // Click terrain to place intersections
    IntersectionEdit,  // Select / delete intersections
    Pathfinding,       // Terrain-based route preview / generation
};
