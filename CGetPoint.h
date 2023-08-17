#pragma once
#include"Tool.h"
#include"ToolMesh.h"
/*--------------------------��������------------------------------*/

template <typename M>
class CGetPointTri
{
public:
	CGetPointTri(int n);  //����n����
	~CGetPointTri() {};
	virtual CPoint2 getPoint() = 0;
	list<CPoint2*>* GetPoints() { return &v_points; } //���ص�
	list<CHalfEdge*>* GetHalfedges() { return &v_halfedge; }  //���ذ��
	list<CEdge*>* GetEdge() { return &v_edge; }  //���ر�


private:
	list<CPoint2*> v_points;  
	list<CHalfEdge*> v_halfedge;
	list<CEdge*> v_edge;
};

template <typename M>
CGetPointTri<M>::CGetPointTri(int n)
{
	char data[100];
	unsigned seed;
	seed = time(0);
	srand(seed);
	double a, b;
	double r, alpha;
	ofstream outfile_vertex;
	outfile_vertex.open("basic.m", ios::out | ios::trunc);
	for (int i = 1; i < 101; i++)
	{
		r = (rand() % 100) / 100;
		alpha = rand() % (2 * PI) - PI;
		a = r * sin(alpha);
		b = r * cos(alpha);
		CPoint2 cpoint(a, b);
		this->v_points.push_back(&cpoint);
		outfile_vertex << "Vertex " << i << " " << a << " " << b << endl;

	}
	outfile_vertex.close();

	ifstream infile_Edge;
	infile_Edge.open("basic.m");
	infile_Edge >> data;

}
