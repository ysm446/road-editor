#pragma once

enum class EditorMode
{
    Navigate,      // Camera only, no editing
    PolylineDraw,  // Click terrain to add road points
    PointEdit,     // Select / drag / delete existing points
};
