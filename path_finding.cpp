#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;

struct sNode {
	int x = 0;
	int y = 0;

	bool bVisited = false;
	bool bTraversable = false;

	float fCost = INFINITY;		// Cost to walk to node, from start
	float fDistance = INFINITY; // Distance to end

	bool operator<(const sNode& n) const {
		return fCost + fDistance < n.fCost + n.fDistance;
	}

	vector<sNode*> vNeighbors;
	sNode* previous = nullptr;
};

// Implementation of A* path finding
int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize) {

	int nMapSize = nMapWidth * nMapHeight;
	sNode* nodes = new sNode[nMapSize];
	sNode* nodeStart = &nodes[nStartY * nMapWidth + nStartX];
	sNode* nodeEnd = &nodes[nTargetY * nMapWidth + nTargetX];

	// Initialize nodes
	for (int i = 0; i < nMapSize; i++) {
		sNode* node = &nodes[i];
		node->bTraversable = pMap[i] == 1;

		// Calculate positions
		int x = i % nMapWidth;
		int y = (i - x) / (nMapHeight + 1);

		node->x = x;
		node->y = y;

		// Add neighbors
		if (y > 0)
			node->vNeighbors.push_back(&nodes[(y - 1) * nMapWidth + x]);
		if (y < nMapHeight - 1)
			node->vNeighbors.push_back(&nodes[(y + 1) * nMapWidth + x]);
		if (x > 0)
			node->vNeighbors.push_back(&nodes[y * nMapWidth + (x - 1)]);
		if (x < nMapWidth - 1)
			node->vNeighbors.push_back(&nodes[y * nMapWidth + (x + 1)]);
	}

	// Using manhattan distance as the heuristic function
	auto heuristic = [=](sNode* a, sNode* b) -> int {
		return (abs(a->x - b->x) + abs(a->y - b->y));
	};

	// Setup starting node and adding it to the open set
	sNode* current = nodeStart;
	current->fCost = 0.0f;
	current->fDistance = heuristic(nodeStart, nodeEnd);

	priority_queue<sNode*> pqOpenSet;
	pqOpenSet.push(current);

	// Open set loop
	while (! pqOpenSet.empty()) {
		current = pqOpenSet.top(); pqOpenSet.pop();
		current->bVisited = true;
		
		// Found path
		if (current == nodeEnd) {
			vector<int> path;
			sNode* p = current;

			//Retrace path.
			while (p->previous != nullptr) {
				int index = p->y * nMapWidth + p->x;
				path.push_back(index);
				p = p->previous;
			}

			// Output path to pOutBuffer if possible
			if (path.size() <= nOutBufferSize) {
				reverse(path.begin(), path.end());
				copy(path.begin(), path.end(), pOutBuffer);
			}
			return path.size();
		}

		// Evaluate neighbors
		for (auto neighbor : current->vNeighbors) {
			if (! neighbor->bTraversable || neighbor->bVisited) {
				continue;
			}
			
			float tentativeCost = current->fCost + heuristic(current, neighbor);
			if (tentativeCost < neighbor->fCost || ! neighbor->bVisited) {
				neighbor->fCost = tentativeCost;
				neighbor->previous = current;
				if (! neighbor->bVisited) {
					pqOpenSet.push(neighbor);
				}
			}
		}
	} 
	// Couldn't find a path.
	return -1;
}

int main() {
	// Example 1: 3 {1,5,9}
	unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];
	cout << FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12) << endl;

	// Example 2: -1
	unsigned char pMap2[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
	int pOutBuffer2[7];
	cout << FindPath(2, 0, 0, 2, pMap2, 3, 3, pOutBuffer2, 7) << endl;
	
	// Example 3: 8 {23, 17, 11, 5, 4, 3, 2, 1}
	unsigned char pMap3[] = { 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	int pOutBuffer3[8];
	cout << FindPath(4, 3, 1, 0, pMap3, 6, 4, pOutBuffer3, 8) << endl;

	// Example 4: 8 {23, 17, 11, 5, 4, 3, 2, 1}
	unsigned char pMap4[] = { 0 };
	int pOutBuffer4[1];
	cout << FindPath(0, 0, 0, 0, pMap4, 1, 1, pOutBuffer4, 1) << endl;

	return 0;
}