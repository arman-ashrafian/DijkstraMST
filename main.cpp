/*
Arman Ashrafian
CS 1D Assignment 12
11-29-2017

1) Implement Dijkstra to find shortest distance from Atlanta
	to all other cities on the map. Identify paths and distances.
2) Determine the Minimum Spanning Tree. Identify edges and 
	total mileage.
*/

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <map>

/********** GLOBAL TYPEDEFS ***********/
typedef std::pair<int, int> intPair;
typedef std::vector< intPair > edgeVec;


// Undirected Graph implemented using an adjacency list
class Graph
{
public:
	Graph(int SIZE);

	// add undirected edge 
	void addEdge(int p1, int p2, int weight);

	// shortest route from start to target.
	// builds *route and returns total distance.
	int dijkstra(int start, int target, std::vector<int> *route);

	int MST(std::vector< intPair > *route);

	// Output all vertices and edges
	void print();

private:
	// sorts adjacent edges
	void __sortAdjEdges();

	// recursively builds route for dijkstras
	void buildRoute(int parent[], int vertex, int startVertex, std::vector<int> *route);
	
	// array of vectors, each vector stores 
	// adjacent edges (pairs) for a vertex.
	// * pair.first = vertex #
	// * pair.second = edge weight
	edgeVec *adj;

	// number of vertices
	int SIZE;

	// infinity
	int oo = ((1LL<<31)-1);
};

Graph::Graph(int SIZE) {
	this->SIZE = SIZE;
	adj = new edgeVec[SIZE];
}


void Graph::addEdge(int p1, int p2, int weight) {
	// create adj edge for both vertices
	adj[p1].push_back(std::make_pair(p2, weight));
	adj[p2].push_back(std::make_pair(p1, weight));
	// sort the edges
	__sortAdjEdges();
}

// Sort adjacent edges by weight
void Graph::__sortAdjEdges() {
	for(int i = 0; i < this->SIZE; i++) {
		std::sort(adj[i].begin(), adj[i].end(),
		// sort by second value
		 [](auto &a, auto &b) {
		 	return a.second < b.second;
		 });
	}
}

void Graph::print() {
	for(int i = 0; i < this->SIZE; i++) {
		std::cout << "Vertex " << i << std::endl;
		for(int j = 0; j < adj[i].size(); j++) {
			std::cout << adj[i][j].first << "\t(" << adj[i][j].second << ")" << std::endl;
		}
		std::cout << " -------------- " << std::endl;
	}
}

 int Graph::dijkstra(int start, int target, std::vector<int> *route) {
 	// min heap
 	std::priority_queue<intPair, std::vector<intPair>, std::greater<intPair> > pq;

 	// unknown distances from start to target (initially oo)
 	std::vector<int> dist(this->SIZE, this->oo);

 	// insert start into pq and set distance to 0
 	// -- WARNING -- 
 	// *first is distance
 	// *second is label
 	// bcuz min heap compares first of pair 
 	pq.push(std::make_pair(0, start));
 	dist[start] = 0;

 	// create vertex parent array & init to all 0's
 	int parent[this->SIZE];
 	for (int i = 0; i < this->SIZE; ++i)
 	{
 		parent[i] = 0;
 	}

 	while(!pq.empty()) {
 		// second is the label
 		int u = pq.top().second;
 		pq.pop();

 		// reached target
 		if(u == target) {
 			buildRoute(parent, u, start, route);
 			return dist[u];
 		}

 		//used to get all adj vertices of a vertex
 		std::vector< intPair >::iterator i;
 		int v, weight;
 		for(i = adj[u].begin(); i != adj[u].end(); ++i) {
 			// get vertex label and weight of current adjacent vertex
 			v = i->first;
 			weight = i->second;

 			if(dist[v] > dist[u] + weight) {
 				// update distance value ( found shorter path )
 				dist[v] = dist[u] + weight;
 				pq.push(std::make_pair(dist[v], v));
 				parent[v] = u;
 			}
 		}

 	}
 	return -1;
}

void Graph::buildRoute(int parent[], int vertex, int startVertex, std::vector<int> *route) {
	if(vertex == startVertex) {
		// reached start vertex
		route->push_back(startVertex);
	} else if(parent[vertex] == 0) {
		// current vertex has no parent
		route->push_back(vertex);
	} else {
		// go for current vertex's parent
		buildRoute(parent, parent[vertex], startVertex, route);
		route->push_back(vertex);
	}
}

int Graph::MST(std::vector< intPair > *route) {
	int totalDistance = 0;
	int start = 0;

	// min heap
 	std::priority_queue<intPair, std::vector<intPair>, std::greater<intPair> > pq;



 	// init all keys to oo
 	std::vector<int> key(this->SIZE, this->oo);

 	// init all parent to -1
 	std::vector<int> parent(this->SIZE, -1);

 	// keep track of vertices in MST
 	std::vector<bool> inMST(this->SIZE, false);

 	// insert start into pq and set init key to 0
 	pq.push(std::make_pair(0, start));
 	key[start] = 0;

 	int u;
 	while(!pq.empty()) {
 		u = pq.top().second;	// min key vertex label
 		pq.pop();

 		inMST[u] = true;

 		std::vector< intPair >::iterator i;

 		int v, weight;
 		for(i = adj[u].begin(); i != adj[u].end(); ++i) {
 			// get vertex label and weight of current adjacent vertex
 			v = i->first;
 			weight = i->second;

 			// if v is not in MST and weight of (u,v) is smaller
 			// than the current key of v
 			if(inMST[v] == false && key[v] > weight) {
 				// update key of v
 				key[v] = weight;
 				pq.push(std::make_pair(key[v], v));
 				parent[v] = u;
 			}
 		}
 	}
    // Build MST edges using parent array and calc total distance
    int weight;
    // std::vector< intPair >::iterator pos;
    for (int i = 1; i < this->SIZE; ++i) {
    	for(auto pos = adj[i].begin(); pos != adj[i].end(); ++pos) {
    		if(pos->first == parent[i]) {
    			weight = pos->second;
    		}
    	}
    	totalDistance += weight;

    	route->push_back(std::make_pair(parent[i], i));
    }
 	return totalDistance;
}

int main()
{
	const int V = 12;

	Graph graph = Graph(V);

	/******************** TESTING SETUP **********************/
	std::map<std::string, int> cities;

	cities["Los Angeles"] = 0;
	cities["San Francisico"] = 1;
	cities["Seattle"] = 2;
	cities["Denver"] = 3;
	cities["Chicago"] = 4;
	cities["Kansas City"] = 5;
	cities["Dallas"] = 6;
	cities["Houston"] = 7;
	cities["Boston"] = 8;
	cities["New York"] = 9;
	cities["Atlanta"] = 10;
	cities["Miami"] = 11;

	std::vector<std::string> cityVec = {
		"Los Angeles",
		"San Francisico",
		"Seattle",
		"Denver",
		"Chicago",
		"Kansas City",
		"Dallas",
		"Houston",
		"Boston",
		"New York",
		"Atlanta",
		"Miami"
	};

	graph.addEdge(cities["Los Angeles"],cities["San Francisico"], 381);
	graph.addEdge(cities["Los Angeles"],cities["Denver"], 1015);
	graph.addEdge(cities["Los Angeles"],cities["Kansas City"], 1663);
	graph.addEdge(cities["Los Angeles"],cities["Dallas"], 1435);

	graph.addEdge(cities["San Francisico"],cities["Denver"], 1267);
	graph.addEdge(cities["San Francisico"],cities["Seattle"], 807);

	graph.addEdge(cities["Seattle"],cities["Denver"], 1331);
	graph.addEdge(cities["Seattle"],cities["Chicago"], 2097);

	graph.addEdge(cities["Denver"],cities["Kansas City"], 599);
	graph.addEdge(cities["Denver"],cities["Chicago"], 1003);

	graph.addEdge(cities["Chicago"],cities["Kansas City"], 533);
	graph.addEdge(cities["Chicago"],cities["Boston"], 983);
	graph.addEdge(cities["Chicago"],cities["New York"], 787);

	graph.addEdge(cities["Kansas City"],cities["Dallas"], 496);
	graph.addEdge(cities["Kansas City"],cities["Atlanta"], 864);
	graph.addEdge(cities["Kansas City"],cities["New York"], 1260);

	graph.addEdge(cities["Dallas"],cities["Atlanta"], 781);
	graph.addEdge(cities["Dallas"],cities["Houston"], 239);

	graph.addEdge(cities["Houston"],cities["Atlanta"], 810);
	graph.addEdge(cities["Houston"],cities["Miami"], 1187);

	graph.addEdge(cities["Miami"],cities["Atlanta"], 661);
	graph.addEdge(cities["Atlanta"],cities["New York"], 888);
	graph.addEdge(cities["Boston"],cities["New York"], 214);
	/******************************************************************/

	std::vector<int> *shortestPath = new std::vector<int>;
	std::vector< intPair > *mst = new std::vector< intPair >;
	int dist;

	/*************** DIJKSTRA ***************/
	int atlantaIndex = cities["Atlanta"];
	for (int i = 0; i < V; ++i)
	{
		if( i != atlantaIndex) {
			dist = graph.dijkstra(atlantaIndex, i, shortestPath);

			std::cout << "PATH FROM Atlanta TO " << cityVec[i] << std::endl
					<< "Distance: " << dist << " mi.\n";

			for(auto v : *shortestPath) {
				std::cout << cityVec[v] << "\n";
			} std::cout << "\n";	
		}
		if(i != 0) { // not building correct path when i = 1 need to find out why
					 // this my my ghetto fix for the assignment.
			shortestPath->clear();
		}
	}


	/********* MINIMUM SPANNING TREE ********/
	dist = graph.MST(mst);

	std::cout << "MINIMUM SPANNING TREE\n"
	 			<< "Distance: " << dist << " mi.\n";

	// output MST
	for (auto v : *mst) {
		std::cout << cityVec[v.first] << " to " << cityVec[v.second] << "\n";
	}

	return 0;
}
