#pragma once
#include<fstream>

#include"Tool.h"
#include"ToolMesh.h"


class pointreader
{
public:
	pointreader(int n);
	~pointreader();
	void MeshWritter();

private:
	list<CPoint2> pointlist;

};

pointreader::pointreader(int n)
{
	char data[100];
	unsigned seed;
	seed = time(0);
	srand(seed);
	double r, alpha;
	double a, b;
	for (int i = 0; i < n; i++)
	{
		r = (rand() % 100) / 100;
		alpha = (rand() % (2 * 314)) / 100;
		a = r * sin(alpha);
		b = r * cos(alpha);
		CPoint2 point(a, b);
		pointlist.push_back(point);
	}

	this->MeshWritter();
}

void pointreader::MeshWritter()
{
	ifstream  fin;
	fin.open("Mesh.m", ios::in);
	list<CPoint2> copyList(pointlist);
	for (int i = 0; i < pointlist.size(); i++)
	{
		//fin << "Vertex " << i<<" " << (copyList.pop_back()).getx()<< " "<< (copyList.pop_back()).getx() << endl;
	}
}

pointreader::~pointreader()
{
}
