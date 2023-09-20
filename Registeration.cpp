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
    FirstRegistered = false;
    
    for (typename M::MeshFaceIterator mv(m_pMesh_N); !mv.end(); mv++)
    {
        CPoint p1 = (*mv)-> halfedge()->target()->point() - (*mv)-> halfedge()->source()->point();
        CPoint p2 = (*mv)-> halfedge()->he_next()->target()->point() - (*mv)-> halfedge()->he_next()->source()->point();
        SN += (p1^p2).norm();
    }
    
    L_N.resize(Vertex_num,Vertex_num);
    W_N.resize(Vertex_num,Vertex_num);
    S_N.resize(Vertex_num,Vertex_num);
    L_M.resize(Vertex_num,Vertex_num);
    W_M.resize(Vertex_num,Vertex_num);
    B.resize(Vertex_num, Vertex_num);
    C.resize(Vertex_num, Vertex_num);
    S_M.resize(Vertex_num,Vertex_num);
    Omega.resize(Vertex_num, Vertex_num);
    V_Omega.resize(Vertex_num, 1);
    I_N.resize(Vertex_num, k);
    I_M.resize(Vertex_num, k);
    f_N.resize(Vertex_num, k);
    f_M.resize(Vertex_num, k);

    lambda_M.resize(k,1);
    lambda_N.resize(k,1);
    
    
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<Eigen::Triplet<double>> triplets_n;
    
    for(int i = 0 ; i < Vertex_num; i++)
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
        y = 0;
        for(j = points.begin();j != points.end(); j++){
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
            ++y;
        }
        ++x;
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
    double asum;
    
    list<CToolVertex*> points;
    points = m_pMesh_M->vertices();
    
    list<CToolVertex*>::iterator i;
    list<CToolVertex*>::iterator j;
    
    for(i = points.begin();i != points.end();i++){
        y = 0;
        for(j = points.begin();j != points.end(); j++){
            num = 0;
            if(x == y){
                asum = 0;
                CTMesh::VertexInHalfedgeIterator velter(m_pMesh_M,*i);
                
                for(;!velter.end();velter++){
                    CPoint p1 = (*velter)->source()->point() -  (*velter)->he_prev()->source()->point();
                    CPoint p2 = (*velter)->he_next()->target()->point() - (*i)->point();
                    CPoint p3 = (*velter)->he_sym()->he_next()->target()->point() - (*i)->point();
                    CPoint p4 = (*velter)->he_sym()->target()->point() - (*velter)->he_sym()->he_next()->target()->point();
                    num += 0.5*(abs((p1*p2)/(p1^p2).norm())+abs((p3*p4)/(p3^p4).norm()));
                    
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
            ++y;
        }
        ++x;
        
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
    
        
    int s = 0;
    for(;evalues.coeff(k-s,0).real()<0.0001 && s < k;s++){}
    if(FirstRegistered){
        s = zeros;
    }else{
        zeros = s;
        FirstRegistered = true;
    }

    I_N.resize(Vertex_num, k+1-s);
    f_N.resize(Vertex_num, k+1-s);
    

    for(int i = 0; i< k+1-s ;i++){
        
        for(int j = 0; j < Vertex_num ;j++){
            tripletList.emplace_back(j,i,(double)(vectors.coeff(j,i).real())/sqrt(evalues.coeff(i,0).real()));
            tripletList_n.emplace_back(j,i,(double)(vectors.coeff(j,i).real()));
        }
        tripletList_e.emplace_back(i,0,(double)evalues.coeff(i,0).real());
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
            typename M::CVertex* pVertex = mv.value();
            if(I_N.coeff(i, j)>0)
            {
                pVertex->rgb()[0] = 1;
            }else{
                pVertex->rgb()[0] = 0;
            }
            i++;
        }
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_N); !mv.end(); mv++)
        {
            typename M::CVertex* pVertex = mv.value();
            
            for(typename M::VertexVertexIterator vv(pVertex);!vv.end();vv++)
            {
                if(pVertex->rgb()[0]+vv.value()->rgb()[0] == 1){
                    pVertex->rgb()[2] = 1;
                    tripletList.emplace_back(i,i,1);
                    
                }
                
            }
            i++;
        }
        
    }
    B.setFromTriplets(tripletList.begin(), tripletList.end());
}

template <typename M>
void Surface<M>::EmBedding_M(){
    std::vector<Eigen::Triplet<double>> tripletList;
    std::vector<Eigen::Triplet<double>> tripletList_n;
    std::vector<Eigen::Triplet<double>> tripletList_e;
    Spectra::SparseGenMatProd<double> op(L_M);
    Spectra::GenEigsSolver<Spectra::SparseGenMatProd<double>> eigs(op, k+1, 2*k+3);
    eigs.init();
    auto nconv = eigs.compute(Spectra::SortRule::SmallestReal);
    Eigen::VectorXcd evalues;
    
    if(eigs.info() == Spectra::CompInfo::Successful)
        evalues = eigs.eigenvalues();
    
    auto vectors = eigs.eigenvectors();
    
    
    int s = 0;
    for(;evalues.coeff(k-s,0).real()<0.0001 && s < k;s++){}
    I_M.resize(Vertex_num, k+1-s);
    f_M.resize(Vertex_num, k+1-s);
    
    for(int i = 0; i< k+1-s ;i++){
        for(int j = 0; j < Vertex_num ;j++){
            tripletList.emplace_back(j,i,(double)(vectors.coeff(j,i).real())/sqrt(evalues.coeff(i,0).real()));
            tripletList_n.emplace_back(j,i,(double)(vectors.coeff(j,i).real()));
        }
        tripletList_e.emplace_back(i,0,(double)evalues.coeff(i,0).real());
    }
    I_M.setFromTriplets(tripletList.begin(), tripletList.end());
    f_M.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
    lambda_M.setFromTriplets(tripletList_e.begin(), tripletList_e.end());
        
    
    tripletList.clear();
    int i;
    for(int j = 0 ; j < k; j++){
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
        {
            typename M::CVertex* pVertex = mv.value();
            if(I_N.coeff(i, j)>0)
            {
                pVertex->rgb()[0] = 1;
            }else{
                pVertex->rgb()[0] = 0;
            }
            i++;

        }
        i = 0;
        for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
        {
            typename M::CVertex* pVertex = mv.value();
            
            for(typename M::VertexVertexIterator vv(pVertex);!vv.end();vv++)
            {
                if(pVertex->rgb()[0]+vv.value()->rgb()[0] == 1){
                    pVertex->rgb()[2] = 1;
                    tripletList.emplace_back(i,i,1);
                }
            }
            C.setFromTriplets(tripletList.begin(), tripletList.end());
            i++;
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
        std::vector<Eigen::Triplet<double>> tripletList_h;
        std::vector<Eigen::Triplet<double>> tripletList_n;
        //equation 16;
        Eigen::SparseMatrix<double> a;
        Eigen::SparseMatrix<double> b;
        a.resize(k,Vertex_num);
        b.resize(k, 1);
        //std::cout<<lambda_N<<endl;
        //std::cout<<lambda_M<<endl;
        //std::cout<<lambda_M-lambda_N<<endl;

        for(int i = 0; i < k;i++){
            for(int j = 0; j< Vertex_num; j++){
                tripletList.emplace_back(i,j,f_N.coeff(j, i)*f_N.coeff(j, i)/S_N.coeff(j, j));
                //tripletList.emplace_back(i,j,f_N.coeff(j, i)/S_N.coeff(j, j));
            }
            tripletList_n.emplace_back(i,0,K*(lambda_N.coeff(i, 0)-lambda_M.coeff(i, 0))/(q*lambda_N.coeff(i, 0)+(K-q)*lambda_M.coeff(i, 0)));
        }
        
        a.setFromTriplets(tripletList.begin(), tripletList.end());
        b.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
        
        
        //equation 19;
        Eigen::SparseMatrix<double> D_Sn = B*I_N - C*I_M;
        Eigen::SparseMatrix<double> E_f(Vertex_num,1);
        
        tripletList.clear();
        for(int i = 0 ;i < Vertex_num; i++)
        {
            double num = 0;
            for(int j = 0; j< k ; j++)
            {
                num += D_Sn.coeff(i, j)*D_Sn.coeff(i, j)*V_Omega.coeff(i,0)/SN;
            }
             
            tripletList.emplace_back(i,0,num);
        }
        E_f.setFromTriplets(tripletList.begin(), tripletList.end());

        
        
        Eigen::SparseMatrix<double> z =  W_N*V_Omega;
        z += E_f;
        
        tripletList.clear();
        tripletList_n.clear();
        tripletList_h.clear();
        Eigen::SparseMatrix<double> h(Vertex_num,1);
        Eigen::SparseMatrix<double> l(Vertex_num,1);
        Eigen::SparseMatrix<double> g(Vertex_num,Vertex_num);
        for(int i = 0; i < Vertex_num; i++) {
            tripletList.emplace_back(i,0,V_Omega.coeff(i,0)+1);
            tripletList_h.emplace_back(i,0,V_Omega.coeff(i,0)-1);
            tripletList_n.emplace_back(i,i,1);
            
        }

        l.setFromTriplets(tripletList_h.begin(),tripletList_h.end());
        h.setFromTriplets(tripletList.begin(), tripletList.end());
        g.setFromTriplets(tripletList_n.begin(), tripletList_n.end());
        
        Eigen::SparseMatrix<double> vOmega = compute(z, E_f, a, b, h, l, g);

        V_Omega = V_Omega + vOmega/(K-q);
        std::cout<<V_Omega<<endl;
        
        tripletList.clear();
        for(int i = 0; i < Vertex_num; i++) {
            tripletList.emplace_back(i,i,V_Omega.coeff(i, 0));
        }
        Omega.setFromTriplets(tripletList.begin(), tripletList.end());
        }

        std::cout<<V_Omega<<endl;
        
    Reg_view();
    

}




template<typename M>
Eigen::SparseMatrix<double> Surface<M>::compute(Eigen::SparseMatrix<double> z,Eigen::SparseMatrix<double> E_f,Eigen::SparseMatrix<double> a,Eigen::SparseMatrix<double> b,Eigen::SparseMatrix<double> h,Eigen::SparseMatrix<double> l,Eigen::SparseMatrix<double> g){
    
    std::vector<Eigen::Triplet<double>> tripletList;

    double eps_abs = 1e-9;

    Eigen::SparseMatrix<double> vOmega(Vertex_num,1);
    

    Eigen::VectorXd z_crowd(Vertex_num);
    Eigen::VectorXd l_crowd(Vertex_num);
    Eigen::VectorXd h_crowd(Vertex_num);
    Eigen::VectorXd b_crowd(k);
    
    for(int i = 0; i < Vertex_num ;i++)
    {
        z_crowd(i,0) = z.coeff(i,0);
        l_crowd(i,0) = l.coeff(i,0);
        h_crowd(i,0) = h.coeff(i,0);
    }
    for(int i = 0;i < k; i++)
    {
        b_crowd(i,0) = b.coeff(i,0);
    }

    sparse::QP<double,int> qp(Vertex_num, k, Vertex_num);

    qp.settings.eps_abs = eps_abs;
    qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    qp.settings.verbose = true;

    qp.init(W_N, z_crowd, a, b_crowd, g, l_crowd, h_crowd);
    qp.solve();

//    std::cout << "primal residual: " << qp.results.info.pri_res << std::endl;
//    std::cout << "dual residual: " << qp.results.info.dua_res << std::endl;
//    std::cout << "total number of iteration: " << qp.results.info.iter
//            << std::endl;
//    std::cout << "setup timing " << qp.results.info.setup_time << " solve time "
//            << qp.results.info.solve_time << std::endl;
    
    Eigen::VectorXd QPSolution = qp.results.x;

    for(int i = 0 ; i < Vertex_num; i ++){
        tripletList.emplace_back(i,0,QPSolution(i,0));
    }
    vOmega.setFromTriplets(tripletList.begin(), tripletList.end());
        
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


