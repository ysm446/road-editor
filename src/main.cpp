#include "App.h"
#include <Windows.h>

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE /*hPrevInstance*/,
                   LPSTR /*lpCmdLine*/, int nCmdShow)
{
    App app;
    if (!app.Initialize(hInstance, nCmdShow))
        return -1;

    return app.Run();
}
