#include <iostream>
#include "plot_utility.h"
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
using namespace std;

int main()
{
	plot_utility gp;
	vector<pair<double,double> > refer_path, refer_path2;
	refer_path.push_back(make_pair(5, 5));
	refer_path.push_back(make_pair(3, 3));
	refer_path2 = refer_path;
	vector<vector<pair<double,double> > > treedata;
	vector<pair<double,double> > limb, limb1;
	limb.push_back(make_pair(3, 2.5));
	limb.push_back(make_pair(5, 4.5));
	limb1.push_back(make_pair(5, 4.5));
	limb1.push_back(make_pair(6, 4));
	treedata.push_back(limb);
	treedata.push_back(limb1);
	vector<pair<double,double> > samp_nodes;
	samp_nodes.push_back(make_pair(4.5, 5));
	samp_nodes.push_back(make_pair(5.5, 5));
	samp_nodes.push_back(make_pair(4.5, 4));
	samp_nodes.push_back(make_pair(5, 5.5));
	gp.plot_tree_for_thesis(refer_path, refer_path2, treedata, samp_nodes, "tree.png");
}
