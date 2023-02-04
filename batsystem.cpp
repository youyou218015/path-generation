
#include "batsystem.h"
#include <fstream>

const double eps = 1e-8;



/**
 * Class constructor for the PSO system starting with empty topology.
 *
 * @param popSize Population size for the PSO system
 * @param iterations Number of iterations while constructing path
 *
 */
BatSystem::BatSystem(int A[][20], double B[][20], int row, int popSize, int iterations)
{
	initTopo(A, B, row);

	std::set<int> uniqueNodes;
	for (auto& edge : edges)
		uniqueNodes.insert({ edge.edgeStart, edge.edgeEnd });
	nodes = uniqueNodes.size();

	init(popSize, iterations);
}

/**
 * Initialises for the Bat system.
 *
 * @param popSize Population size for the Bat system
 * @param iterations Number of iterations while constructing path
 * @Param fmin\fmax Frequency from the initialization
 */
void BatSystem::init(int popSize, int iterations)
{
	if (popSize > 0 && iterations > 0)
	{
		this->popSize = popSize;
		this->iterations = iterations;
	}
	else
	{
		this->popSize = POPULATION_SIZE;
		this->iterations = ITERATIONS;
	}

	std::random_device rd;
	gen = std::mt19937_64(rd());
}

/**
 * Constructor for bats.
 */
Bat::Bat()
{
	id = ++counter;
	fitness = bestFitness = 10000;
	cost = 0;
}

/**
 *  Destructor.
 */
BatSystem::~BatSystem() { }

/**
 * Finds the path between src and dest.
 *
 * @param src Originating node
 * @param dest Destination node of the request
 * @return std::vector<int> Node container representing the path
 */
std::vector<int> BatSystem::path(int src, int dest)
{
	if (nodes <= 1)
		return std::vector<int>();

	// Initialise all particles in their vector (velocity & position)
	initBats(src, dest);

	// For each bat, construct path and store its fitness value
	// Iterate and update pBest and nBest in each particle

	int i = 1;                    // iteration number variable

	std::vector<int> bestPath;   //  used to store the best path

	bool breakflag = false;

	for (auto& bat : bats)
	{
		constructPath(bat);
		updateFitness(bat);

	}

	int t = 1;
	do
	{
		// update the GBest position of the swarm
		updateGBest();
		updateWorst();

		// Reposition particles
		for (auto& bat : bats)
		{
			// Move bat into the search space
			updateVelocity(bat);
			double ave = aveLoudness();
			updatePosition(bat, ave);



			acceptNewSol(bat, i);

			// Construct path and evaluate fitness of each particle
			constructPath(bat);
			updateFitness(bat);

			if (abs(bat.fitness - (UPPER_VAL - LOWER_VAL)) <= eps)         //abs
		//	if (abs(bat.fitness - (UPPER_VAL - LOWER_VAL) * (UPPER_VAL - LOWER_VAL)) <= eps || (bat.fitness <= (UPPER_VAL - LOWER_VAL) * (UPPER_VAL - LOWER_VAL)))         //square
			{


				std::cout << "fitness_value: " << bat.fitness << std::endl;

				std::cout << "cost: " << bat.cost << std::endl;

				std::ofstream fout("C:\\Users\\dylen\\Desktop\\result.txt", std::ios_base::out | std::ios_base::app);
				fout << "iter_num: " << i << "   ";
				fout.close();

				bestPath = bat.path;
				breakflag = true;
				break;

			}

			t++;
		}

		if (breakflag) break;

	} while (i++ < iterations);



	if (i > iterations)
	{

		std::ofstream fout("C:\\Users\\dylen\\Desktop\\result.txt", std::ios_base::out | std::ios_base::app);
		fout << "Failed, iteration number:  " << i << "   ";
		fout.close();

	}

	if (breakflag) return bestPath;
	else
	{
		double tempValue = std::numeric_limits<double>::max();                 // minimization problem
		bestPath.clear();
		for (auto& bat : bats)
			if (bat.bestFitness < tempValue)
			{
				tempValue = bat.bestFitness;
				bestPath = bat.bestPath;
			}

		return bestPath;
	}

}

/**
 * Initialises the particle population.
 *
 * @param src Originating node
 * @param dest Destination node of the request
 */
void BatSystem::initBats(int src, int dest)
{
	bats.clear();
	std::uniform_int_distribution<> distro(-VELOCITY_INIT_LIMIT, VELOCITY_INIT_LIMIT);
	std::uniform_real_distribution<> disA(Amin, A0);
	std::uniform_real_distribution<> disR(Rmin, Rmax);

	// Priorities and velocities
	int i = 0;
	while (i++ < popSize)
	{
		// bat creation
		Bat tempBat;
		generatePri(tempBat, src, dest);       // it can be viewed as initilization position

		for (int j = 0; j < nodes; ++j)
			tempBat.velocityVec.push_back(distro(gen));

		// initialize the Loudness
		for (int k = 0; k < nodes; ++k)
			tempBat.nodeLoudness.push_back(disA(gen));

		// initialize the Pulase
		for (int k = 0; k < nodes; ++k)
			tempBat.nodePulse.push_back(disR(gen));

		tempBat.fitness = 10000;
		tempBat.bestFitness = 10000;

		bats.push_back(tempBat);                             //std::vector<Particle> particles;
	}
}

/**
 * Generates the bat priority vector.
 *
 * @param particle The particle to generate its node priority vector
 * @param src Originating node
 * @param dest Destination node of the request
 */
void BatSystem::generatePri(Bat& bat, int src, int dest)
{
	bat.nodePriVec.clear();
	std::uniform_int_distribution<> distro(-PRIORITY_INIT_LIMIT, PRIORITY_INIT_LIMIT);

	// Topology nodes mapped to priorities
	bat.nodePriVec.push_back({ src, distro(gen) });
	for (int node = 0; node < nodes; ++node)
		if (node != src && node != dest)
			bat.nodePriVec.push_back({ node, distro(gen) });
	bat.nodePriVec.push_back({ dest, distro(gen) });
}

/**
 * Constructs the internal path of the given particle.
 *
 * @param particle This particle's path that will be constructed
 */
void BatSystem::constructPath(Bat& bat)
{
	std::vector<int> path;
	std::vector<std::pair<int, double>> npCopy(bat.nodePriVec);   
	path.push_back((*npCopy.cbegin()).first);                          

	// used to store the neighbour nodes that have direct connections with the current node
	std::vector<std::pair<int, double>> adjacentnodes;

	//auto iter_current_node = path.begin();
	auto iter_current_node = path.end() - 1;

	while (!isValid((*npCopy.cbegin()).first, (*npCopy.crbegin()).first, path))
	{
		auto iter = npCopy.begin();
		adjacentnodes.clear();    // in order to store new

		while (iter != npCopy.end())
		{
			if (isNeighbour(*path.crbegin(), (*iter).first))
				adjacentnodes.push_back(*iter);
			++iter;
		}

		// choose the node with the highest priority
		auto it = adjacentnodes.begin();

		int highestPri = (int)(*it).second;
		int highestPriNode = (*it).first;

		while (it != adjacentnodes.end())
		{
			auto ndTemp = (*it).first;           //node 
			auto priTemp = (*it).second;         // priority
			if (priTemp > highestPri)
			{
				highestPri = (int)priTemp;
				highestPriNode = ndTemp;
			}
			++it;
		}
		path.push_back(highestPriNode);

	}

	bat.path = path;
}

/**
 * Updates bat's velocity.
 *
 * @param bat The bat to update its velocity
 */
void BatSystem::updateVelocity(Bat& bat)
{
	if (bat.worstVal != bat.gbestVal)
		bat.scoreVal = (double)(bat.worstVal - bat.fitness) / (bat.worstVal - bat.gbestVal);
	else bat.scoreVal = 1;

	std::uniform_real_distribution<> distro(0, 1);

	for (int i = 0; i < nodes; ++i)
	{
		double beta = distro(gen);

		double f = fmin + (fmax - fmin) * beta;  // frequency used to update the velocity

		//use neighbour best to update
	//	double tempVelo = (double)(bat.nodePriVec.at(i).second - bat.nBestVec.at(i)) * f;

		//use global best to update
		double tempVelo = (double)(bat.nodePriVec.at(i).second - bat.GBestVec.at(i).second) * f;

	//	double velocity = bat.velocityVec.at(i) + tempVelo;
		double velocity = bat.velocityVec.at(i)*(1-bat.scoreVal) + tempVelo;

		// Clamp velocity
		if (velocity > VELOCITY_LIMIT)
			velocity = VELOCITY_LIMIT;
		if (velocity < -VELOCITY_LIMIT)
			velocity = -VELOCITY_LIMIT;

		bat.velocityVec[i] = velocity;
	}
}

/**
 * Updates bat's position.
 *
 * @param bat The bat to update its position
 */
void BatSystem::updatePosition(Bat& bat, double aveLoudness)
{

	for (int i = 0; i < nodes; ++i)
		bat.nodePriVec[i].second = bat.nodePriVec[i].second + bat.velocityVec.at(i);

	std::uniform_real_distribution<> distro(Rmin, Rmax);
	std::uniform_real_distribution<> disepsilon(-1, 1);

	for (int i = 0; i < nodes; ++i)
	{
		if (distro(gen) > bat.nodePulse.at(i))
		{
			bat.nodePriVec[i].second = bat.GBestVec.at(i).second+ disepsilon(gen) * aveLoudness;
		}
	}
}

/**
 * compute the average loudness.
 */
double BatSystem::aveLoudness()
{
	double loudsum = 0.0;
	for (auto& bat : bats)
	{
		for (int i = 0; i < nodes; ++i)
			loudsum += bat.nodeLoudness.at(i);
	}

	return (double)(loudsum / (bats.size() * nodes));
}

/**
 * Updates bat's loudness.
 *
 * @param bat The bat to update its position
 */
void BatSystem::updateLoudness(Bat& bat, int i)
{
	bat.nodeLoudness.at(i) = bat.nodeLoudness.at(i) * Alpha;
}

/**
 * Updates bat's pulse.
 *
 * @param bat The bat to update its pulse
 */
void BatSystem::updatePulse(Bat& bat, int i, int iter)
{
	bat.nodePulse.at(i) = bat.nodePulse.at(i) * (1 - exp(-Gamma * iter));
}


// Determing whether accept the new solution or not
void BatSystem::acceptNewSol(Bat& bat, int iter)
{
	std::uniform_real_distribution<> disR(Amin, A0);

	for (int i = 0; i < nodes; ++i)
	{
		int cond1 = (disR(gen) < bat.nodeLoudness.at(i)) ? true : false;
		int cond2 = (bat.fitness < bat.bestFitness) ? true : false;
		if (cond1 && cond2)
		{
			updateLoudness(bat, i);
			updatePulse(bat, i, iter);
		}
	}

}

/**
 * Checks the validity of the given path.
 *
 * @param src Path's source node
 * @param dest Path's destination node
 * @param path The node path to validate
 * @return bool Validity indicator
 */
bool BatSystem::isValid(int src, int dest, std::vector<int>& path) const
{
	// Ensure path's src and dest are correct
	return (*path.begin() == src && (isLoop(path) || *path.rbegin() == dest)) ? true : false;
}

/**
 * Checks the whether there exist a loop of the given path.
 *
 * @param path The node path to validate
 * @return bool Validity indicator
 */
bool BatSystem::isLoop(std::vector<int>& path) const
{
	// Ensure path's src and dest are correct

	for (auto start1 = path.begin(); start1 != path.end(); start1++)
	{
		for (auto start2 = start1 + 1; start2 != path.end(); start2++)
		{
			if (*start1 == *start2) return true;
		}
	}

	return  false;
}

/**
 * Checks if the given nodes are neighbours.
 *
 * @param a The first node
 * @param b The second node
 * @return bool True if nodes are neighbours
 */
bool BatSystem::isNeighbour(int a, int b) const    //for directed digraph
{

	for (auto& edge : edges)
		if (edge.edgeStart == a && edge.edgeEnd == b)
			return true;

	return false;
}

/**
 * Clears the system.
 */
void BatSystem::clear()
{
	edges.clear();
	bats.clear();
	nodes = 0;
}

/**
 * Used for producing particle IDs.
 */
long int Bat::counter = 0;

/**
 * Inserts an edge.
 *
 * @param src Source node
 * @param dest Destination node
 * @param weight Weight for the edge
 */
void BatSystem::insertEdge(int src, int dest, double weight)
{
	AdaptiveSystem::insertEdge(src, dest, weight);
	std::set<int> uniqueNodes;
	for (auto& edge : edges)
		uniqueNodes.insert({ edge.edgeStart, edge.edgeEnd });
	nodes = uniqueNodes.size();
}

/**
 * Updates particle's fitness. In this case, the path cost.
 *
 * @param particle The particle to update its fitness
 */
void BatSystem::updateFitness(Bat& bat)
{
	// It should include a valid path
	if (!bat.path.size() || bat.path.size() == 1)
	{
		bat.fitness = 10000;
		return;
	}

	if (!isLoop(bat.path))   //  no loop path
	{
		// For every path's edge, find its cost
		double cost = 0;
		int i = 0;
		std::vector<int>::const_iterator it = bat.path.cbegin();
		while (it++ != bat.path.cend() - 1)
		{
			int edgeStart = *(it - 1);
			int edgeEnd = *it;

			auto tempIt = std::find_if(edges.cbegin(), edges.cend(), \
				[edgeStart, edgeEnd](Edge edge)
				{
					return edge.edgeStart == edgeStart && edge.edgeEnd == edgeEnd;
				});
			if (tempIt == edges.cend())
				throw std::invalid_argument("\nBatSystem::updateFitness... Edge not found");

			cost += (*tempIt).weight * pow(GAMMA_VAL, i);
			++i;
		}
		bat.cost = cost;

		bat.fitness = abs(cost - LOWER_VAL) + abs(cost - UPPER_VAL);
	//	bat.fitness = (cost - LOWER_VAL)*(cost - LOWER_VAL) + (cost - UPPER_VAL)*(cost - UPPER_VAL);

	}
	else    // exists loop
	{
		double cost = 0, tmp = 0, costtmp = 0;
		int i = 0, j = 0;
		std::vector<int>::const_iterator it = bat.path.cbegin();
		std::vector<int>::const_iterator itend = bat.path.cend() - 1;
		int loopnode = *itend;

		// for the non-loop part
		while (*it != loopnode)
		{

			int edgeStart = *(it);
			int edgeEnd = *(it + 1);

			auto tempIt = std::find_if(edges.cbegin(), edges.cend(), \
				[edgeStart, edgeEnd](Edge edge)
				{
					return edge.edgeStart == edgeStart && edge.edgeEnd == edgeEnd;
				});
			if (tempIt == edges.cend())
				throw std::invalid_argument("\nBatSystem::updateFitness... Edge not found");

			cost += (*tempIt).weight * pow(GAMMA_VAL, i);
			++i;
			++it;
		}

		// for the loop part
		if (*it == loopnode)
		{
			while (it != bat.path.cend() - 1)
			{
				int edgeStart = *(it);
				int edgeEnd = *(it + 1);

				auto tempIt = std::find_if(edges.cbegin(), edges.cend(), \
					[edgeStart, edgeEnd](Edge edge)
					{
						return edge.edgeStart == edgeStart && edge.edgeEnd == edgeEnd;
					});
				if (tempIt == edges.cend())
					throw std::invalid_argument("\nBatSystem::updateFitness... Edge not found");

				tmp += (*tempIt).weight * pow(GAMMA_VAL, j);
				++j;
				++it;
			}

			if (tmp != 0)
			{
				costtmp = tmp * pow(GAMMA_VAL, i) * (1 / (1 - pow(GAMMA_VAL, j)));
			}
		}

		// cost = non_loop_cost + loop_cost
		cost = cost + costtmp;

		bat.cost = cost;
		bat.fitness = abs(cost - LOWER_VAL) + abs(cost - UPPER_VAL);
	//	bat.fitness = (cost - LOWER_VAL) * (cost - LOWER_VAL) + (cost - UPPER_VAL) * (cost - UPPER_VAL);
	}
}


/**
 * Comparison of current instance with the rhs, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of current id being less than rhs'
 */
bool Bat::operator<(const Bat& rhs) const
{
	return id < rhs.id;
}

/**
 * Comparison of current instance with the rhs, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of current id being greater than rhs'
 */
bool Bat::operator>(const Bat& rhs) const
{
	return id > rhs.id;
}

/**
 * Comparison of current instance with the rhs for equality, based on ids.
 *
 * @param rhs The right-hand side object
 * @return bool The indication of equality
 */
bool Bat::operator==(const Bat& rhs) const
{
	return id == rhs.id;
}


// used to update the best position of the Batsystem,   together with the gBest Value
void BatSystem::updateGBest()
{
	std::vector<std::pair<int, double>> GBestVec;
	std::vector<std::pair<int, double>> tmpGBestVec;

	double tempValue = std::numeric_limits<double>::max();
	for (auto& bat : bats)
	{
		if (bat.bestFitness < tempValue)
		{
			tempValue = bat.bestFitness;
			tmpGBestVec = bat.nodePriVec;
		}
	}

	for (auto& bat : bats)
	{
		bat.GBestVec = tmpGBestVec;
		bat.gbestVal = tempValue;
	}

}

// used to update the worst position of the Batsystem,   together with the worst Value
void BatSystem::updateWorst()
{
	std::vector<std::pair<int, double>> WorstVec;
	std::vector<std::pair<int, double>> tmpWorstVec;

	double tmpValue = std::numeric_limits<double>::min();
	for (auto& bat : bats)
	{
		if (bat.fitness > tmpValue)
		{
			tmpValue = bat.fitness;
			tmpWorstVec = bat.nodePriVec;
		}
	}

	for (auto& bat : bats)
	{
		bat.WorstVec = tmpWorstVec;
		bat.worstVal = tmpValue;
	}

}





