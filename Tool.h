#ifndef _TOOL_H_
#define _TOOL_H_

#include<vector>
#include<math.h>
#include<cstdlib>
#include<fstream>  

#include "ToolMesh.h"

#ifndef PI
#define PI 3.1415926535
#endif


namespace MeshLib
{
	using namespace std;

	template<typename M>
	class CTool
	{
	public:
		CTool(M* pMesh);
		~CTool(){};

		void test();
		void _change_color();
	protected:
		M* m_pMesh;
	};

	template<typename M>
	CTool<M>::CTool(M* pMesh)
	{
		m_pMesh = pMesh;
	}

	template<typename M>
	void CTool<M>::test()
	{


		int lambda = 0;
		int v = (int)m_pMesh->numVertices();
		int F = (int)m_pMesh->numFaces();
		int E = (int)m_pMesh->numEdges();
		lambda = F - E + v;
		cout << "ŷ��ʾ������" <<lambda<< endl;
		cout  << "ŷ��ʾ����*2PI=" << lambda * PI * 2 << endl;
		int edgeNum = 0;
		
		m_pMesh->vertices();
		list<CToolVertex*> points;
		list<CEdge*> eList;
		points = m_pMesh->vertices();
		list<CToolVertex*>::iterator i;
		list<CToolVertex*>::iterator j;
		double k_v = 0;
		double alpha = 0;
		CHalfEdge* h;
		for (i = points.begin();i !=points.end();i++)
		{
			if ((*i)->boundary())
			{
				CTMesh::VertexEdgeIterator velter(*i);
				int edgeNum = 0;
				for (; !velter.end(); velter++)
				{
					edgeNum++;
				}
				k_v += PI - (PI * edgeNum) / 3;
			}
			else
			{
				CTMesh::VertexEdgeIterator velter(*i);
				int edgeNum = 0;
				for (; !velter.end(); velter++)
				{
					edgeNum++;
				}
				k_v += 2*PI - (PI * edgeNum) / 3;
			}
		}
		cout << "��˹���ʺ�Ϊ��" << k_v << "    ��Ϊ�� " << k_v - 2 * PI * lambda << endl;
	}

	template<typename M>
	void CTool<M>::_change_color()
	{
		for (typename M::MeshVertexIterator mv(m_pMesh); !mv.end(); mv++)
		{
			typename M::CVertex* pVertex = mv.value();
			pVertex->rgb()[0] = 1;
			pVertex->rgb()[1] = 1;
			pVertex->rgb()[2] = 0;
		}
	}
	
}


#endif