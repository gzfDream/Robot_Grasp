//----------------------------------------------------------------------
//		File:			nearestPoint.cpp
//		Programmer:		gzf
//		Last modified:	04/26/2018 (Release 0.1)
//		Description:	寻找模型上点到骨架上点的最近点集
//----------------------------------------------------------------------

//----------------------------------------------------------------------

#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <ANN/ANN.h>					// ANN declarations
#include <vector>

using namespace std;					// make std:: accessible

struct Point_info
{
	int index;
	double x;
	double y;
	double z;
	double distance;
};

bool readPt(istream &in, ANNpoint p, int dim)			// read point (false on EOF)
{
	for (int i = 0; i < dim; i++) {
		if (!(in >> p[i])) return false;
	}
	return true;
}

void printPt(ostream &out, ANNpoint p, int dim)			// print point
{
	out << "(" << p[0];
	for (int i = 1; i < dim; i++) {
		out << ", " << p[i];
	}
	out << ")\n";
}

bool searchPoints(int dim, int maxPts, int K_per, double eps, string data, string query) {
	pair<int, Point_info> map;//mesh上点距离最近的骨架点，int骨架点序号，point_infomesh点信息
	vector<pair<int, Point_info>> vec;

	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	static ifstream dataStream;					// data file stream
	static ifstream queryStream;				// query file stream

	istream*		dataIn = NULL;			// input for data points
	istream*		queryIn = NULL;			// input for query points

	// open data file
	dataStream.open(data, ios::in);
	if (!dataStream) {
		cerr << "Cannot open data file\n";
		return false;
	}
	dataIn = &dataStream;

	// open query file
	queryStream.open(query, ios::in);
	if (!queryStream) {
		cerr << "Cannot open query file\n";
		return false;
	}
	queryIn = &queryStream;			// make this query stream


	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[K_per];					// allocate near neigh indices
	dists = new ANNdist[K_per];					// allocate near neighbor dists

	nPts = 0;									// read data points

	cout << "Data Points:\n";
	while (nPts < maxPts && readPt(*dataIn, dataPts[nPts], dim)) {
		printPt(cout, dataPts[nPts], dim);
		nPts++;
	}

	kdTree = new ANNkd_tree(					// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	int num = 0;
	while (readPt(*queryIn, queryPt, dim)) {			// read query points
		//cout << "Query point: ";				// echo query point
		//printPt(cout, queryPt, dim);

		kdTree->annkSearch(						// search
			queryPt,						// query point
			K_per,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		Point_info p;
		p.index = num;
		p.distance = dists[0];
		p.x = queryPt[0];
		p.y = queryPt[1];
		p.z = queryPt[2];

		map.first = nnIdx[0];
		map.second = p;
		vec.push_back(map);

		num++;

		//cout << "\tNN:\tIndex\tDistance\n";
		//for (int i = 0; i < K_per; i++) {			// print summary
		//	dists[i] = sqrt(dists[i]);			// unsquare distance
		//	cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
		//}
	}
	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();									// done with ANN

	for (int i = 0; i < vec.size(); i++)
	{
		if (vec[i].first == 0)
			cout << vec[i].second.x <<" "<< vec[i].second.y <<" "<< vec[i].second.z << endl;
	}

	return true;
}

int main(int argc, char **argv)
{
	int				k = 1;			// number of nearest neighbors
	int				dim = 3;			// dimension
	double			eps = 0;			// error bound
	int				maxPts = 1000;			// maximum number of data points
	
	searchPoints(dim, maxPts, k, eps, "../../models/dog.txt","../../models/dogPoint.txt");

	system("pause");
	return EXIT_SUCCESS;
}

