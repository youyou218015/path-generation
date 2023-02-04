
#include "adaptivesystem.h"

/**
 * Constructor for edges.
 */
AdaptiveSystem::Edge::Edge()
{
	edgeStart = edgeEnd = id = 0;
	weight = 0;
}

/**
 * Comparison of current instance with the rhs, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of current id being less than rhs'
 */
bool AdaptiveSystem::Edge::operator<(const AdaptiveSystem::Edge& rhs) const
{
	return id < rhs.id;
}

/**
 * Comparison of current instance with the rhs, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of current id being greater than rhs'
 */
bool AdaptiveSystem::Edge::operator>(const AdaptiveSystem::Edge& rhs) const
{
	return id > rhs.id;
}

/**
 * Comparison of current instance with the rhs for equality, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of equality
 */
bool AdaptiveSystem::Edge::operator==(const AdaptiveSystem::Edge& rhs) const
{
	return id == rhs.id;
}

/**
 * Constructor for the AdaptiveSystem.
 */
AdaptiveSystem::AdaptiveSystem() { }

/**
 * Destructor for the AdaptiveSystem.
 */
AdaptiveSystem::~AdaptiveSystem() { }


void AdaptiveSystem::initTopo(int A[][20], double B[][20], int row)
{
	int src = 0; int dest = 0; double weight = 0.0;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < row; ++j)
		{
			if (A[i][j] == 1)
			{
				src = i;
				dest = j;
				weight = B[i][j];
				insertEdge(src, dest, weight);
			}
		}
	}
}

/**
 * Inserts an edge.
 *
 * @param src Source node
 * @param dest Destination node
 * @param weight Weight for the edge
 */
void AdaptiveSystem::insertEdge(int src, int dest, double weight) noexcept(false)
{
	AdaptiveSystem::Edge edge;
	edge.edgeStart = src;
	edge.edgeEnd = dest;
	edge.weight = weight;
	edge.id = ++edgeIdCnt;
	edges.push_back(edge);
}

/**
 * Used for producing edge IDs.
 */
int AdaptiveSystem::edgeIdCnt = 0;
