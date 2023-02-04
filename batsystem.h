
#ifndef BATSYSTEM_H
#define BATSYSTEM_H

#include <iostream>
#include <vector>
#include <set>
#include <exception>
#include <memory>
#include <functional>
#include <random>
#include <limits>
#include <cmath>
#include "adaptivesystem.h"

struct Bat
{
	Bat();
	static long int counter;
	long int id;
	std::vector<std::pair<int, double>> nodePriVec;
	std::vector<double> velocityVec;
	std::vector<std::pair<int, double>> GBestVec;
	std::vector<std::pair<int, double>> WorstVec;
//	std::vector<double> nBestVec;
//	std::vector<double> pBestVec;
	std::vector<double> nodeLoudness;
	std::vector<double> nodePulse;

	std::vector<int> path;
	std::vector<int> bestPath;
	double fitness;
	double bestFitness;
	double cost;
	double gbestVal;
	double worstVal;
	double scoreVal;

	bool operator<(const Bat&) const;
	bool operator>(const Bat&) const;
	bool operator==(const Bat&) const;

};


class BatSystem : public AdaptiveSystem
{
public:
	static const int POPULATION_SIZE = 30;
	static const int ITERATIONS = 150;
	static const int VELOCITY_LIMIT = 3000;
	static const int VELOCITY_INIT_LIMIT = 10;
	static const int PRIORITY_INIT_LIMIT = 100;

	static constexpr double fmin = 0;
	static constexpr double fmax = 1;
	static constexpr double A0 = 0.9;
	static constexpr double Amin = 0;
	static constexpr double Rmin = 0;
	static constexpr double Rmax = 0.25;
	static constexpr double Gamma = 0.9;
	static constexpr double Alpha = 0.9;

	static constexpr double GAMMA_VAL = 0.9;       // discounted coefficient
	static constexpr double LOWER_VAL = 5;       //  low bound of the interval
	static constexpr double UPPER_VAL = 20;      //  upper bound of the interval

	BatSystem(int A[][20], double B[][20], int, int, int);
	virtual ~BatSystem();                                            //why use virtual £¿£¿
	virtual std::vector<int> path(int, int);
	virtual void clear();
	virtual void insertEdge(int, int, double) noexcept(false);

protected:
	virtual void updateFitness(Bat&) noexcept(false);

private:
	void init(int, int);
	void initBats(int, int);
	void generatePri(Bat&, int, int);
	void constructPath(Bat&);
	bool isValid(int, int, std::vector<int>&) const;
	bool isNeighbour(int, int) const;
	bool isLoop(std::vector<int>& path) const;
	void updateVelocity(Bat&);
	void updatePosition(Bat&, double aveLoudness);

//	void updatePositionHeuristic(Bat&);            //using heuristic information to update the position

	void updateLoudness(Bat&, int);
	void updatePulse(Bat&, int, int);
	double aveLoudness();

	void updateGBest();
	void updateWorst();
	void acceptNewSol(Bat&, int);        //  Determing whether accept the new solution or not 

//	void updatePBest(Bat&);
//	void updateNBest(Bat&);
//	Bat getPreviousNeighbour(Bat&) const noexcept(false);         //use struct Particle£¿£¿
//	Bat getNextNeighbour(Bat&) const noexcept(false);
//	std::vector<int> bestPath() const;

//	std::vector<double> GBestPos() const;    //  used to store the GBest of the whole swarm
	std::vector<Bat> bats;
	int nodes;
	int popSize;
	int iterations;
	std::mt19937_64 gen;

};

#endif // BATSYSTEM_H
