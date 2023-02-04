
#ifndef ADAPTIVESYSTEM_H
#define ADAPTIVESYSTEM_H

#include <functional>
#include <vector>
#include <string>

class AdaptiveSystem
{
public:
	struct Edge
	{
		Edge();
		int edgeStart;
		int edgeEnd;
		double weight;
		long int id;
		bool operator<(const AdaptiveSystem::Edge&) const;
		bool operator>(const AdaptiveSystem::Edge&) const;
		bool operator==(const AdaptiveSystem::Edge&) const;
	};

	AdaptiveSystem();
	virtual ~AdaptiveSystem();
	virtual std::vector<int> path(int, int) = 0;
	virtual void insertEdge(int, int, double) noexcept(false);
	virtual void clear() = 0;                                     // =0 ???


protected:
	virtual void initTopo(int A[][20], double B[][20], int row);
	std::vector<AdaptiveSystem::Edge> edges;

private:
	static int edgeIdCnt;
};

#endif // ADAPTIVESYSTEM_H
