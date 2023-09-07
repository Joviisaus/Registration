#include "Registeration.h"
#include <math.h>
#include <iostream>


 
namespace MeshLib
{
template <typename M>
Surface<M>::Surface(M* pMesh_N ,M* pMesh_M ,int k){
    SN = 0;
    this->k = k;
    m_pMesh_N = pMesh_N;
    m_pMesh_M = pMesh_M;
    Vertex_num = (int)m_pMesh_M->numVertices();
    
    for (typename M::MeshFaceIterator mv(m_pMesh_N); !mv.end(); mv++)
    {
        CPoint p1 = (*mv)-> halfedge()->target()->point() - (*mv)-> halfedge()->source()->point();
        CPoint p2 = (*mv)-> halfedge()->he_next()->target()->point() - (*mv)-> halfedge()->he_next()->source()->point();
        SN += (p1^p2).norm();
    }
    
    L_N.resize(Vertex_num+1,Vertex_num+1);
    W_N.resize(Vertex_num+1,Vertex_num+1);
    S_N.resize(Vertex_num+1,Vertex_num+1);
    L_M.resize(Vertex_num+1,Vertex_num+1);
    W_M.resize(Vertex_num+1,Vertex_num+1);
    B.resize(Vertex_num+1, Vertex_num+1);
    C.resize(Vertex_num+1, Vertex_num+1);
    S_M.resize(Vertex_num+1,Vertex_num+1);
    Omega.resize(Vertex_num+1, Vertex_num+1);
    V_Omega.resize(Vertex_num+1, 1);
    I_N.resize(Vertex_num+1, k);
    I_M.resize(Vertex_num+1, k);
    f_N.resize(Vertex_num+1, k);
    f_M.resize(Vertex_num+1, k);

    lambda_M.resize(k,1);
    lambda_N.resize(k,1);
    
    
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<Eigen::Triplet<double>> triplets_n;
    
    for(int i = 0 ; i < Vertex_num+1; i++)
    {
        triplets.emplace_back(i,i,1.0f);
        triplets_n.emplace_back(i,0,1.0f);
    }
    Omega.setFromTriplets(triplets.begin(),triplets.end());
    V_Omega.setFromTriplets(triplets_n.begin(), triplets_n.end());
    
}

template <typename M>
void Surface<M>::buildLBM_N(){
    std::vector<Eigen::Triplet<double>> triplets ;
    std::vector<int> r;
    std::vector<int> c;
    std::vector<double> val;
    
    std::vector<Eigen::Triplet<double>> triplets_S ;
    std::vector<int> r_S;
    std::vector<int> c_S;
    std::vector<double> val_S;
    
    int x = 0;
    int y = 0;
    
    double num;
    double asum;
    
    list<CToolVertex*> points;
    points = m_pMesh_N->vertices();
    
    list<CToolVertex*>::iterator i;
    list<CToolVertex*>::iterator j;
    
    for(i = points.begin();i != points.end();i++){
        
        ++x;
        y = 0;
        for(j = points.begin();j != points.end(); j++){
            ++y;
            num = 0;
            if(x == y){
                
                asum = 0;
                CTMesh::VertexInHalfedgeIterator velter(m_pMesh_N,*i);
                
                for(;!velter.end();velter++){
                    CPoint p1 = (*velter)->source()->point() -  (*velter)->he_prev()->source()->point();
                    CPoint p2 = (*velter)->he_next()->target()->point() - (*i)->point();
                    CPoint p3 = (*velter)->he_sym()->he_next()->target()->point() -  (*i)->point();
                    CPoint p4 = (*velter)->he_sym()->target()->point() - (*velter)->he_sym()->he_next()->target()->point();
                    num += 0.5*(abs((p1*p2)/(p1^p2).norm())+abs((p3*p4)/(p3^p4).norm()));
                    
                    asum += abs((p1^p2).norm());
                    
                }
                r_S.push_back(x);
                c_S.push_back(x);
                val_S.push_back(1/asum);
                
                
            }else{
                CTMesh::VertexInHalfedgeIterator velter(m_pMesh_N,*i);
                for(;!velter.end();velter++){
                    if(((*velter)->source())==(*j)){
                        CPoint p1 = (*velter)->source()->point() -  (*velter)->he_prev()->source()->point();
                        CPoint p2 = (*velter)->he_next()->target()->point() - (*i)->point();
                        CPoint p3 = (*velter)->he_sym()->he_next()->target()->point() -  (*i)->point();
                        CPoint p4 = (*velter)->he_sym()->target()->point() - (*velter)->he_sym()->he_next()->target()->point();
                        num = -0.5*(abs((p1*p2)/(p1^p2).norm())+abs((p3*p4)/(p3^p4).norm()));
                        break;
                    }
                }
                
            }
            if(num != 0){
                r.push_back(x);
                c.push_back(y);
                val.push_back(num);
            }
            
        }
        

    }
    
    for(int p = 0; p < r.size(); p++){
        triplets.emplace_back(r[p],c[p],val[p]);
    }
    
    for(int p = 0; p < r_S.size(); p++){
        triplets_S.emplace_back(r_S[p],c_S[p],val_S[p]);
    }
    
    L_N.setFromTriplets(triplets.begin(),triplets.end());
    S_N.setFromTriplets(triplets_S.begin(),triplets_S.end());
    W_N = S_N*L_N;
    
    
    
    
}


template <typename M>
void Surface<M>::buildLBM_M(){
    std::vector<Eigen::Triplet<double>> triplets ;
    std::vector<int> r;
    std::vector<int> c;
    std::vector<double> val;
    
    std::vector<Eigen::Triplet<double>> triplets_S ;
    std::vector<int> r_S;
    std::vector<int> c_S;
    std::vector<double> val_S;
    
    int x = 0;
    int y = 0;
    
    double num;
    double row;
    double asum;
    
    list<CToolVertex*> points;
    points = m_pMesh_M->vertices();
    
    list<CToolVertex*>::iterator i;
    list<CToolVertex*>::iterator j;
    
    for(i = points.begin();i != points.end();i++){
        ++x;
        y = 0;
        row = 0;
        for(j = points.begin();j != points.end(); j++){
            ++y;
            num = 0;
            asum = 0;
            if(x == y){
                
                CTMesh::VertexInHalfedgeIterator velter(m_pMesh_M,*i);
                
                for(;!velter.end();velter++){
                    CPoint p1 = (*velter)->source()->point() -  (*velter)->he_prev()->source()->point();
                    CPoint p2 = (*velter)->he_next()->target()->point() - (*i)->point();
                    CPoint p3 = (*velter)->he_sym()->he_next()->target()->point() - (*i)->point();
                    CPoint p4 = (*velter)->he_sym()->target()->point() - (*velter)->he_sym()->he_next()->target()->point();
                    num += -0.5*((p1*p2)/(p1^p2).norm()+(p3*p4)/(p3^p4).norm());
                    
                    asum += abs((p1^p2).norm());
                }
                r_S.push_back(x);
                c_S.push_back(x);
                val_S.push_back(1/asum);
                
                
            }else{
                CTMesh::VertexInHalfedgeIterator velter(m_pMesh_M,*i);
                for(;!velter.end();velter++){
                    if(((*velter)->source())==(*j)){
                        CPoint p1 = (*velter)->source()->point() -  (*velter)->he_prev()->source()->point();
                        CPoint p2 = (*velter)->he_next()->target()->point() - (*i)->point();
                        CPoint p3 = (*velter)->he_sym()->he_next()->target()->point() - (*i)->point();
                        CPoint p4 = (*velter)->he_sym()->target()->point() - (*velter)->he_sym()->he_next()->target()->point();
                        num = 0.5*((p1*p2)/(p1^p2).norm()+(p3*p4)/(p3^p4).norm());
                        break;
                    }
                }
                
            }
            if(num != 0){
                r.push_back(x);
                c.push_back(y);
                val.push_back(num);
                row += num;
            }
            
        }
        
        
    }
    
    for(int p = 0; p < r.size(); p++){
        triplets.emplace_back(r[p],c[p],val[p]);
    }
    
    for(int p = 0; p < r_S.size(); p++){
        triplets_S.emplace_back(r_S[p],c_S[p],val_S[p]);
    }
    
    L_M.setFromTriplets(triplets.begin(),triplets.end());
    S_M.setFromTriplets(triplets_S.begin(),triplets_S.end());
    W_M = S_M*L_M;
    
    
}

template <typename M>
void Surface<M>::EmBedding_N(){
    std::vector<Eigen::Triplet<double>> tripletList;
    std::vector<Eigen::Triplet<double>> tripletList_n;
    std::vector<Eigen::Triplet<double>> tripletList_e;
    Spectra::SparseGenMatProd<double> op(Omega*L_N);
    Spectra::GenEigsSolver<Spectra::SparseGenMatProd<double>> eigs(op, k+1, 2*k+3);
    eigs.init();
    auto nconv = eigs.compute(Spectra::SortRule::SmallestReal);
    Eigen::VectorXcd evalues;
    
    if(eigs.info() == Spectra::CompInfo::Successful)
        evalues = eigs.eigenvalues();
    
    auto vectors = eigs.eigenvectors();
    
    //std::cout << evalues <<endl;
        
    
    for(int i = 1; i<= k ;i++){
        
        for(int j = 1; j <= Vertex_num ;j++){
            tripletList.emplace_back(j,i-1,(double)(vectors.coeff(j,i).real())/(1+sqrt(evalues.coeff(0,i).real())));
            tripletList_n.emplace_back(j,i-1,(double)(vectors.coeff(j,i).real()));
        }
        tripletList_n.emplace_back(i-1,0,(double)evalues.coeff(i).real());
    }
    I_N.setFromTriplets(tripletList.begin(), tripletList.end());
    f_N.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
    lambda_N.setFromTriplets(tripletList_e.begin(), tripletList_e.end());
    
    tripletList.clear();
    int i;
        for(int j = 0 ; j < k; j++){
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_N); !mv.end(); mv++)
        {
            i++;
            typename M::CVertex* pVertex = mv.value();
            if(I_N.coeff(i, j)>0)
            {
                pVertex->rgb()[0] = 1;
            }else{
                pVertex->rgb()[0] = 0;
            }
        }
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_N); !mv.end(); mv++)
        {
            i++;
            typename M::CVertex* pVertex = mv.value();
            
            for(typename M::VertexVertexIterator vv(pVertex);!vv.end();vv++)
            {
                if(pVertex->rgb()[0]+vv.value()->rgb()[0] == 1){
                    pVertex->rgb()[2] = 1;
                    tripletList.emplace_back(i,i,1);
                    
                }
                
            }
            
        }
        
    }
    B.setFromTriplets(tripletList.begin(), tripletList.end());
}

template <typename M>
void Surface<M>::EmBedding_M(){
    std::vector<Eigen::Triplet<double>> tripletList;
    std::vector<Eigen::Triplet<double>> tripletList_n;
    std::vector<Eigen::Triplet<double>> tripletList_e;
    Spectra::SparseGenMatProd<double> op(W_M);
    Spectra::GenEigsSolver<Spectra::SparseGenMatProd<double>> eigs(op, k+1, 2*k+3);
    eigs.init();
    auto nconv = eigs.compute(Spectra::SortRule::SmallestReal);
    Eigen::VectorXcd evalues;
    
    if(eigs.info() == Spectra::CompInfo::Successful)
        evalues = eigs.eigenvalues();
    
    auto vectors = eigs.eigenvectors();
    
    //std::cout << evalues <<endl << vectors<<endl;
    
    for(int i = 1; i<= k ;i++){
        for(int j = 1; j <= Vertex_num ;j++){
            tripletList.emplace_back(j,i-1,(double)(vectors.coeff(j,i).real())/sqrt(evalues.coeff(0,i).real()));
            tripletList_n.emplace_back(j,i-1,(double)(vectors.coeff(j,i).real()));
        }
        tripletList_n.emplace_back(i-1,0,(double)evalues.coeff(i).real());
    }
    I_N.setFromTriplets(tripletList.begin(), tripletList.end());
    f_N.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
    lambda_N.setFromTriplets(tripletList_e.begin(), tripletList_e.end());
    
    
    tripletList.clear();
    int i;
    for(int j = 0 ; j < k; j++){
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
        {
            i++;
            typename M::CVertex* pVertex = mv.value();
            if(I_N.coeff(i, j)>0)
            {
                pVertex->rgb()[0] = 1;
            }else{
                pVertex->rgb()[0] = 0;
            }
        }
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
        {
            i++;
            typename M::CVertex* pVertex = mv.value();
            
            for(typename M::VertexVertexIterator vv(pVertex);!vv.end();vv++)
            {
                if(pVertex->rgb()[0]+vv.value()->rgb()[0] == 1){
                    pVertex->rgb()[2] = 1;
                    tripletList.emplace_back(i,i,1);
                }
            }
            C.setFromTriplets(tripletList.begin(), tripletList.end());
        }     

    }
}

template <typename M>
void Surface<M>::Registeration(int K){
    Eigen::MatrixXd Omega_v(Omega.rows(),1);
    for(int i = 0; i < Omega.rows(); i++) Omega_v(i,0) = Omega.coeff(i, i);
    EmBedding_M();
    for(int q = 0; q < K ; q++){
        EmBedding_N();
        std::vector<Eigen::Triplet<double>> tripletList;
        std::vector<Eigen::Triplet<double>> tripletList_n;
        //equation 16;
        Eigen::SparseMatrix<double> a;
        Eigen::SparseMatrix<double> b;
        a.resize(k,Vertex_num+1);
        b.resize(k, 1);
        for(int i = 0; i < k;i++){
            for(int j = 0; j< Vertex_num+1; j++){
                tripletList.emplace_back(i,j,f_N.coeff(j, i)*f_N.coeff(j, i)/S_N.coeff(j, j));
            }
            tripletList.emplace_back(i,0,K*(lambda_N.coeff(i, 0)-lambda_M.coeff(i, 0))/(q*lambda_N.coeff(i, 0)+(K-q)*lambda_M.coeff(i, 0)));
        }
        
        a.setFromTriplets(tripletList.begin(), tripletList.end());
        b.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
        
        
        //equation 19;
        Eigen::SparseMatrix<double> D_Sn = B*I_N - C*I_M;
        Eigen::SparseMatrix<double> E_f(Vertex_num+1,1);
        
        tripletList.clear();
        for(int i = 0 ;i < Vertex_num+1; i++)
        {
            double num = 0;
            for(int j = 0; j< k ; j++)
            {
                num += D_Sn.coeff(i, j)*D_Sn.coeff(i, j);
            }
             
            tripletList.emplace_back(i,0,num);
        }
        E_f.setFromTriplets(tripletList.begin(), tripletList.end());
        
        Eigen::SparseMatrix<double> z =  W_N*V_Omega;
        z += E_f;
        
        tripletList.clear();
        tripletList_n.clear();
        Eigen::SparseMatrix<double> h(2*(Vertex_num+1),1);
        Eigen::SparseMatrix<double> g(2*(Vertex_num+1),Vertex_num+1);
        for(int i = 0; i < Vertex_num +1; i++) {
            tripletList.emplace_back(i,0,V_Omega.coeff(i,0)+1);
            tripletList_n.emplace_back(i,i,-1);
        }
        for(int i = Vertex_num +1; i < 2*(Vertex_num +1); i++) {
            tripletList.emplace_back(i,0,1-V_Omega.coeff(i-(Vertex_num +1),0));
            tripletList_n.emplace_back(i,i-(Vertex_num +1),1);
        }
        
        h.setFromTriplets(tripletList.begin(), tripletList.end());
        g.setFromTriplets(tripletList_n.begin(), tripletList_n.end());

        
        Eigen::SparseMatrix<double> vOmega = compute(z, E_f, a, b, h, g);
        //std::cout<<vOmega<<endl;

        V_Omega = V_Omega + vOmega/(K-q);
        
        tripletList.clear();
        for(int i = 0; i < Vertex_num +1; i++) {
            tripletList.emplace_back(i,i,V_Omega.coeff(i, 0));
        }
        Omega.setFromTriplets(tripletList.begin(), tripletList.end());
        }

        std::cout<<V_Omega<<endl;
        
    Reg_view();
    

}



template<typename M>
Eigen::SparseMatrix<double> Surface<M>::compute(Eigen::SparseMatrix<double> z,Eigen::SparseMatrix<double> E_f,Eigen::SparseMatrix<double> a,Eigen::SparseMatrix<double> b,Eigen::SparseMatrix<double> h,Eigen::SparseMatrix<double> g){
    
    std::vector<Eigen::Triplet<double>> tripletList;
    
    double epsilon = 1e-9;
    Eigen::SparseMatrix<double> vOmega(Vertex_num+1,1);
    Eigen::SparseMatrix<double> dvOmega(Vertex_num+1,1);
    Eigen::SparseMatrix<double> WO(Vertex_num+1,1);
    Eigen::SparseMatrix<double> WOZ(Vertex_num+1,1);
    vOmega.setZero();
    //从初始化开始限制好av=b,
    z = 2*z;
    /*
     TODO：
     实现二次线性规划
     */
    for(int i = 0;i < 1000; i++)
    {
        //这一部分用牛顿迭代法求
        tripletList.clear();
        WO = W_N*vOmega;
        WOZ = W_N*vOmega - z;
        if(WO.norm()!=0) WO /= WO.norm();
        if(WOZ.norm()!=0) WOZ /= WOZ.norm();
        
        if((W_N*vOmega - z).norm()< 0.01) break;
        for(int j = 0; j < Vertex_num+1; j++)
        {
            
            if( vOmega.coeff(j,0) == 0 || WO.coeff(j,0) == 0){
                tripletList.emplace_back(j,0,-WOZ.coeff(j,0));
            }else {
                tripletList.emplace_back(j,0,-WOZ.coeff(j,0)/(WO.coeff(j,0)/vOmega.coeff(j,0)));
            }

        }
        dvOmega.setFromTriplets(tripletList.begin(),tripletList.end());
        
        WO = g*(vOmega+dvOmega);
        for(int j = 0; j < Vertex_num+1; j++)
        {
            if(WO.coeff(j,0)>h.coeff(j,0)) return vOmega;
        }
        //std::cout<<dvOmega<<endl;

        vOmega += dvOmega;

        //接下来的这一部分处理av = b
        for(int j = 0;j< k;j++)
        {
            tripletList.clear();
            for(int s = 0; s < Vertex_num +1; s++)
            {
                if(a.coeff(j,s) == 0){
                    tripletList.emplace_back(s,0,vOmega.coeff(s,0));
                }else{
                    tripletList.emplace_back(s,0,(a.coeff(j,s)*vOmega.coeff(s,0)-b.coeff(j,0))/a.coeff(j,s));
                }
                
            }
            vOmega.setFromTriplets(tripletList.begin(),tripletList.end());

        }


    }
        
    return vOmega;
    
}



template<typename M>
void Surface<M>::_change_color()
{
    int i = 0;
    for (typename M::MeshVertexIterator mv(m_pMesh_N); !mv.end(); mv++)
    {
        i++;
        typename M::CVertex* pVertex = mv.value();
        if(I_N[0][i]>0)
        {
            pVertex->rgb()[0] = 1;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }else if (I_N[0][i]<0){
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 1;
        }else{
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }
    }
}

template<typename M>
void Surface<M>::_change()
{
    int i = 0;
    for (typename M::MeshVertexIterator mv(m_pMesh_N); !mv.end(); mv++)
    {
        i++;
        typename M::CVertex* pVertex = mv.value();
        if(I_N.coeff(i,1)>0 && I_N.coeff(i,2)>0)
        {
            pVertex->rgb()[0] = 1;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }else if (I_N.coeff(i,1)<0 && I_N.coeff(i,2)>0){
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 1;
        }else if (I_N.coeff(i,1)<0 && I_N.coeff(i,2)<0){
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 1;
            pVertex->rgb()[2] = 0;
        }else{
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }
    }
}

template<typename M>
void Surface<M>::Reg_view()
{
    int i = 0;
    for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
    {
        i++;
        typename M::CVertex* pVertex = mv.value();
        if(Omega.coeff(i, i) > 1.1)
        {
            pVertex->rgb()[0] = 1;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }else if (Omega.coeff(i, i) < 0.9){
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 1;
        }else{
            pVertex->rgb()[0] = 0;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }
    }
}
}


