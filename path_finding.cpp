#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
using namespace std;

struct sNode {
	int x;
	int y;

	bool bVisited = false;
	bool bTraversable = false;

	float fCost = INFINITY;
	float fDistance = INFINITY;

	sNode* previous = nullptr;
	vector<sNode*> vNeighbors;
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

	//Initialize nodes
	for (int i = 0; i < nMapSize; i++) {
		nodes[i].bTraversable = pMap[i] == 1;
		nodes[i].x = i % nMapWidth;
		nodes[i].y = i / (nMapHeight+1);
	}

	//Initialize nodes' neighbors
	for (int i = 0; i < nMapSize; i++) {
		sNode* node = &nodes[i];
		int x = node->x;
		int y = node->y;

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
	auto heuristic = [](sNode* a, sNode* b) {
		return (abs(a->x - b->x) + abs(a->y - b->y));
	};

	//Setup starting node and adding it to the open set.
	sNode* current = nodeStart;
	current->fCost = 0.0f;
	current->fDistance = heuristic(nodeStart, nodeEnd);
	priority_queue<sNode*> pqOpenSet;
	pqOpenSet.push(current);

	//Open set loop
	while (! pqOpenSet.empty() && current != nodeEnd) {

		while (! pqOpenSet.empty() && pqOpenSet.top()->bVisited)
			pqOpenSet.pop();

		if (pqOpenSet.empty())
			break;

		current = pqOpenSet.top();
		current->bVisited = true;

		for (auto neighbor : current->vNeighbors) {
			if (!neighbor->bVisited && neighbor->bTraversable) {
				pqOpenSet.push(neighbor);
			}

			float tentativeCost = current->fCost + heuristic(current, neighbor);

			if (tentativeCost < neighbor->fCost) {
				neighbor->previous = current;
				neighbor->fCost = tentativeCost;
				neighbor->fDistance = neighbor->fCost + heuristic(neighbor, nodeEnd);
			}
		}
	}

	// Calculate and return distance from path
	if (current == nodeEnd) {
		sNode* p = nodeEnd;
		vector<int> cords;

		while (p->previous != nullptr) {
			int index = p->y * nMapWidth + p->x;
			cords.push_back(index);
			p = p->previous;
		}

		if (cords.size() < nOutBufferSize) { //Output path to pOutBuffer if possible.
			reverse(cords.begin(), cords.end());
			copy(cords.begin(), cords.end(), pOutBuffer);
		}
		return cords.size();
	}
	// Couldn't find a path.
	return -1;
}

int main() {
	//Example 1: 3 {1,5,9}
    unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
    int pOutBuffer[12];
	cout << FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12) << endl;

	//Example 2: -1
	unsigned char pMap2[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
	int pOutBuffer2[7];
	cout << FindPath(2, 0, 0, 2, pMap2, 3, 3, pOutBuffer2, 7) << endl;
    return 0;
}