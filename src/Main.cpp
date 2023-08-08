#include <graphics/Application.h>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
    return cg::Application(new cg::MainWindow(1280,760)).run(argc, argv);
}
