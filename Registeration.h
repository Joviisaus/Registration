#ifndef _TOOL_H_
#define _TOOL_H_


#include<vector>
#include<math.h>
#include<cstdlib>
#include<fstream>
#include<Eigen/SparseLU>
#include<Spectra/GenEigsSolver.h>
#include<Spectra/MatOp/SparseGenMatProd.h>
#include<Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"


#include "ToolMesh.h"

#ifndef PI
#define PI 3.1415926535
#endif

namespace MeshLib
{
	using namespace std;

	
	template<typename M>
	class Surface
	{
	public:
		Surface(M* pMesh_N,M* pMesh_M,int k);
		~Surface(){};
        void buildLBM_N();
        void buildLBM_M();
		void EmBedding_N();
        void EmBedding_M();
        void updateOMEGA();
        void Registeration(int K);
        void _change_color();
        void Reg_view();
        void _change();
        std::vector<vector<double>> getI();
		int getVertex_num();
        
	protected:
		M* m_pMesh_N;
        M* m_pMesh_M;
        int k;
        int zeros;
        bool FirstRegistered;
        Eigen::SparseMatrix<double> L_N;
        Eigen::SparseMatrix<double> W_N;
        Eigen::SparseMatrix<double> S_N;
        Eigen::SparseMatrix<double> I_N;
        Eigen::SparseMatrix<double> f_N;
        Eigen::SparseMatrix<double> B;
        Eigen::SparseMatrix<double> lambda_N;
        
        Eigen::SparseMatrix<double> L_M;
        Eigen::SparseMatrix<double> W_M;
        Eigen::SparseMatrix<double> S_M;
        Eigen::SparseMatrix<double> I_M;
        Eigen::SparseMatrix<double> f_M;
        Eigen::SparseMatrix<double> C;
        Eigen::SparseMatrix<double> lambda_M;
                
        Eigen::SparseMatrix<double> Omega;
        Eigen::SparseMatrix<double> V_Omega;
        
        Eigen::SparseMatrix<double> compute(Eigen::SparseMatrix<double> z,Eigen::SparseMatrix<double> E_f,Eigen::SparseMatrix<double> a,Eigen::SparseMatrix<double> b,Eigen::SparseMatrix<double> h,Eigen::SparseMatrix<double> g);
 
        
        int Vertex_num;
        double SN;
        
	};


}

#endif
