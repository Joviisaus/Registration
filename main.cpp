#include<iostream>

#include"Registeration.h"
#include"Registeration.cpp"


using namespace std;
using namespace MeshLib;

#define N 100;


int main(int argc, char** argv)
{
    CTMesh mesh_N;
    CTMesh mesh_M;
    mesh_N.read_m("../Model/eight.m");
    mesh_M.read_obj("../Model/eight_Reg.obj");
    mesh_M.write_m("../Model/eight_Reg.m");
    Surface<CTMesh> Surface(&mesh_N,&mesh_M,8);
    Surface.buildLBM_N();
    Surface.buildLBM_M();
    cout << "LBM build finished" <<endl;
    Surface.Registeration(3);
    Surface.Reg_view();
    mesh_N.write_m("../Model/eight_osqp.m");
    cout << "finished !!! press any key to continue!!!" << endl;
    getchar();
    
    return 0;

}
