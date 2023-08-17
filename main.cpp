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
    //mesh.read_m(argv[1]);
    mesh_N.read_m("../../Model/iso_eight.m");
    mesh_M.read_m("../../Model/eight.m");
    Surface<CTMesh> Surface(&mesh_N,&mesh_M,5);
    Surface.buildLBM_N();
    Surface.buildLBM_M();
    cout << "LBM build finished" <<endl;
    Surface.Registeration(10);
    //mesh.write_m(argv[2]);
    //Surface._change_color();
    mesh_M.write_m("../../Model/eight_Reg.m");
    cout << "finished !!! press any key to continue!!!" << endl;
    getchar();
    
    return 0;

}
