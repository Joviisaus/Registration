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
    mesh_N.read_m("../../Model/loveme.m");
    mesh_M.read_m("../../Model/loveme.m");
    Surface<CTMesh> Surface(&mesh_N,&mesh_M,10);
    Surface.buildLBM_N();
    Surface.buildLBM_M();
    Surface.EmBedding_N();
    cout << "LBM build finished" <<endl;
    //Surface.Registeration(10);
    //mesh.write_m(argv[2]);
    //Surface._change_color();
    Surface._change();
    mesh_N.write_m("../../Model/loveme_23.m");
    cout << "finished !!! press any key to continue!!!" << endl;
    getchar();
    
    return 0;

}
