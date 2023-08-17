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

        std::cout<<"computting.."<<endl;
        
        Eigen::SparseMatrix<double> vOmega = compute(z, E_f, a, b, h, g);
        
        V_Omega = V_Omega + vOmega/(K-q);
        
        tripletList.clear();
        for(int i = 0; i < Vertex_num +1; i++) {
            tripletList.emplace_back(i,i,V_Omega.coeff(i, 0));
        }
        Omega.setFromTriplets(tripletList.begin(), tripletList.end());
        std::cout<<q*10+10<<"% finished/n"<<endl;
        }
        
    Reg_view();
    

}



template<typename M>
Eigen::SparseMatrix<double> Surface<M>::compute(Eigen::SparseMatrix<double> z,Eigen::SparseMatrix<double> E_f,Eigen::SparseMatrix<double> a,Eigen::SparseMatrix<double> b,Eigen::SparseMatrix<double> h,Eigen::SparseMatrix<double> g){
    
    std::vector<Eigen::Triplet<double>> tripletList;
    
    double epsilon = 1e-9;
    Eigen::SparseMatrix<double> vOmega(Vertex_num+1,1);
    vOmega.setZero();
    
//    for(int i = 0; i< Vertex_num+1 ;i++) tripletList.emplace_back(i,0,0);
//    vOmega.setFromTriplets(tripletList.begin(), tripletList.end());
//    tripletList.clear();
//    
    for(int i = 0; i< k; i++){
        for(int j = 0 ;j < Vertex_num +1; j++){
            if(a.coeff(i, j) == 0) continue;
            vOmega.insert(j, 0) = vOmega.coeff(j, 0) + b.coeff(i,0)/a.coeff(i, j);
        }
    }
    std::vector<double> asIndex;
    Eigen::MatrixXd Sk = Eigen::MatrixXd::Zero(Vertex_num+1, 1);
    Eigen::MatrixXd bi_left(g.rows(), vOmega.cols());
    bi_left = g * vOmega;
    Eigen::MatrixXd Aee(1, 1);
    
    for (int i = 0; i < Vertex_num+1; i++) {
            double bb = h.coeff(i, 0);
            if (bi_left(i, 0) <= h.coeff(i, 0) + epsilon) {
                Sk(i, 0) = 1;}
            }


    int iterTimes = 0;
    while (iterTimes < 100) {
        int activeSetSize = 0;
        for (int i = 0; i < Sk.rows(); i++) {
            if (Sk(i) == 1) {
                activeSetSize += 1;
            }
        }
        if (k != 0) {
            Aee.resize(k + activeSetSize, a.cols());
            Aee.block(0, 0, a.rows(), a.cols()) = a;
            int aee_index = 0;
            for (int i = 0; i < Vertex_num+1; i++) {
                if (Sk(i, 0) == 1) {
                    Aee.block(k + aee_index, 0, 1, a.cols()) = g.row(i);
                    asIndex.push_back(i);
                    aee_index += 1;
                }
            }
        }
        else {
            Aee.resize(activeSetSize, g.cols());
            int aee_index = 0;
            for (int i = 0; i < Vertex_num+1; i++) {
                if (Sk(i, 0) == 1) {
                    Aee.block(aee_index, 0, 1, g.cols()) = g.row(i);
                    asIndex.push_back(i);
                    aee_index += 1;
                }
            }
        }
        
        Eigen::MatrixXd d = Eigen::MatrixXd::Zero(vOmega.rows(), 1);
        Eigen::MatrixXd lambda = Eigen::MatrixXd::Zero(Aee.rows(), 1);
        Eigen::MatrixXd gk;
        gk = W_N * vOmega + z;
        Eigen::MatrixXd bee = Eigen::MatrixXd::Zero(Aee.rows(), 1);
        
        //qp_lagrange(W_N, gk, Aee, bee, d, lambda, d.rows(), Aee.rows());
        
        double d_norm = 0;
        for (int i = 0; i < d.rows(); i++) {
            d_norm += d(i) * d(i);
        }
        
        // d的模是否=0 是 停算
        if (d_norm < 1e-6) {
            // 终止判断
            double minLambda = 9999;
            int minLambdaIndex = 0;
            for (int i = 0; i < lambda.rows(); i++) {
                if (lambda(i, 0) < minLambda) {
                    minLambda = lambda(i, 0);
                    minLambdaIndex = i;
                }
            }
            if (minLambda > 0) {
                break;
            }
            // 从有效集中剔除约束
            else {
                int removed_cons = asIndex[minLambdaIndex];
                //int removed_cons = asIndex[minLambdaIndex - k];
                Sk(removed_cons, 0) = 0;
            }
        }
        // 否 求步长
        else {
            Eigen::MatrixXd ad(g.rows(), 1);
            ad = g * d;
            Eigen::MatrixXd ax(g.rows(), 1);
            ax = g * vOmega;
            std::vector<double> alphaList;
            int min_alpha_index = 0;
            double min_alpha = 9999;
            std::cout<<"iterTimes --+"<<endl;
            for (int i = 0; i < Sk.rows(); i++) {
                if (Sk(i) == 0 && ad(i, 0) < 0) {
                    double alpha_ = (h.coeff(i, 0) - ax(i, 0)) / ad(i, 0);
                    if (alpha_ < min_alpha) {
                        min_alpha = alpha_;
                        min_alpha_index = i;
                    }
                    alphaList.push_back(alpha_);
                }
            }
            std::cout<<min_alpha<<endl;
            if (min_alpha > 1) {
                vOmega = vOmega + d;
            }
            else {
                vOmega = vOmega + min_alpha * d;
                Sk(min_alpha_index) = 1;    // 将最小的alpha对应的约束加入有效集
            }
        }
        
        iterTimes += 1;
        
    }
    std::cout<<iterTimes<<" while end"<<endl;
    double num = 0;
    for(int i = 0 ; i < Vertex_num+1; i++){
        std::cout<<vOmega.coeff(i,0)<< " " << vOmega.cols()<<" "<<vOmega.rows()<<endl;
        num += vOmega.coeff(i,0);
    }
    std::cout<<num<<endl;

    return vOmega;
    
}

template<typename M>
void Surface<M>::qp_lagrange(Eigen::SparseMatrix<double> H, Eigen::MatrixXd& c, Eigen::MatrixXd& A, Eigen::MatrixXd& b,
                 Eigen::MatrixXd& x, Eigen::MatrixXd& lambda, const int& dim, const int& m) {
    Eigen::MatrixXd G(dim, dim);
    Eigen::MatrixXd B(m, dim);
    Eigen::MatrixXd C(m, m);
    
    //Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    //solver.compute(H);
    Eigen::MatrixXd I(H.cols(),H.rows());
    //I.setIdentity();
    I.setZero();
    auto H_inv = I;

    
    G = H_inv - H_inv * A.transpose() * (A * H_inv * A.transpose()).inverse() * A * H_inv;
    B = (A * H_inv * A.transpose()).inverse() * A * H_inv;
    C = -1 * (A * H_inv * A.transpose()).inverse();
    
    x = B.transpose() * b - G * c;
    lambda = B * c - C * b;
    
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
void Surface<M>::Reg_view()
{
    int i = 0;
    for (typename M::MeshVertexIterator mv(m_pMesh_M); !mv.end(); mv++)
    {
        i++;
        typename M::CVertex* pVertex = mv.value();
        if(Omega.coeff(i, i) > 1.0001)
        {
            pVertex->rgb()[0] = 1;
            pVertex->rgb()[1] = 0;
            pVertex->rgb()[2] = 0;
        }else if (Omega.coeff(i, i) < 0.9999){
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


