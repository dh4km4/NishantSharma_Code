/*
Author: Nishant Sharma
Class: ECE4122
Last Date Modified: Oct 11 2021

Description:
This program solves the ant & seed problem where
an ant roams randomly in a 5x5 grid starting at the 
center. 5 seeds are placed in the lowest row, and if the
ant stumbles onto a lower row and finds a seed it carries it
till it reaches an empty square in the topmost row where it drops it
The number of steps the ant takes should converge if the algorithm is run enough times. 
The algorithm is run 10,000,000 times using multiple threads, and 
the average is computed to 6 decimal places.
*/

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <random>
#include <vector>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <fstream>

using namespace std;

#define UP 1
#define DOWN 2
#define RIGHT 3
#define LEFT 4

unsigned long int totalSteps = 0;
mutex totalStepsMtx;

/*
The struct for a coordinate containing x and y
to keep track of the position of the ant and seeds
in the 5x5 Grid
*/
struct Coordinate
{
	int x;
	int y;
};

/*
The struct for a seed containing a coordinate
to hold the position of the seed and a boolean 
indicating if the seed has reached the topmost row
*/
struct Seed
{
	Coordinate position;
	bool reachedTop;
};

/*
Input Parameters:- 
	num; takes in the number of times to run the algorithm
The function that runs the algorithm 'n' number 
of times specified and adds the number of steps 
for 'n' runs to the global variable totalSteps
*/
void runAlgorithm(int num)
{
	// Initial Setup for each thread
	Coordinate antPosition = { 0, 0 };
	Seed seedPositions[5];
	int seedsReachedTop = 0;
	unsigned long int steps = 0;
	Seed* carriedSeed;

	random_device myRandomDevice;
	int seed = myRandomDevice();
	default_random_engine generator(seed);
	uniform_int_distribution<int> distribution(1, 4);
	for (int i = 0; i < num; i++)
	{
		antPosition.x = 0;
		antPosition.y = 0;
		for (int k = -2; k < 3; k++)
		{
			seedPositions[k + 2] = { {k, -2}, false };
		}
		seedsReachedTop = false;
		carriedSeed = NULL;
		while (seedsReachedTop < 5)
		{
			/*
			distribution(generator) -- Choose up, down, right or left based on position
			If position is invalid (causes ant to go off grid) find a new position
			*/
			bool valid = false;  
			int direction = 0;
			while (!valid)
			{
				direction = distribution(generator);
				Coordinate finalPosition;
				switch (direction)
				{
				case UP:
					finalPosition = { antPosition.x, antPosition.y + 1 };
					if (finalPosition.y <= 2)
					{
						valid = true;
					}
					break;
				case DOWN:
					finalPosition = { antPosition.x, antPosition.y - 1 };
					if (finalPosition.y >= -2)
					{
						valid = true;
					}
					break;
				case RIGHT:
					finalPosition = {antPosition.x + 1, antPosition.y};
					if (finalPosition.x <= 2)
					{
						valid = true;
					}
					break;
				case LEFT:
					finalPosition = { antPosition.x - 1, antPosition.y };
					if (finalPosition.x >= -2)
					{
						valid = true;
					}
					break;
				}
				if (valid)
				{
					antPosition.x = finalPosition.x;
					antPosition.y = finalPosition.y;
				}
			}
			if (carriedSeed) // Ant is carrying a seed
			{
				if (antPosition.y == 2) // Check to see if ant is at top row
				{
					bool seedAtPosition = false;
					for (int i = 0; i < 5; i++)
					{
						// Check if the square empty
						if (seedPositions[i].position.x == antPosition.x && seedPositions[i].position.y == antPosition.y)
						{
							seedAtPosition = true;
						}
					}
					if (!seedAtPosition)
					{
						// If ant is at top and position empty "drop the seed"
						carriedSeed->position.x = antPosition.x;
						carriedSeed->position.y = antPosition.y;
						if (carriedSeed->position.y == 2)
						{
							carriedSeed->reachedTop = true;
							seedsReachedTop++;
						}
						carriedSeed = NULL;
					}
				}
			}
			else // Ant is not carrying seed
			{
				for (int i = 0; i < 5; i++)
				{
					// Check if a seed exists at position and is not at top
					if (antPosition.y < 2 && seedPositions[i].position.x == antPosition.x && seedPositions[i].position.y == antPosition.y)
					{
						// "Carry the seed" at the position
						carriedSeed = &seedPositions[i];
					}
				}
			}
			steps += 1; // Update number of steps
		}
	}
	totalStepsMtx.lock();
	totalSteps += steps;
	totalStepsMtx.unlock();
}


int main()
{
	unsigned long numThreads = thread::hardware_concurrency();
	int num = 10000000;
	int runPerThread = (int)ceil((double)num / numThreads);
	vector<thread> threads;
	for (int i = 0; i < numThreads; i++)
	{
		threads.push_back(thread(runAlgorithm, runPerThread));
	}
	for (auto& th : threads)
	{
		th.join();
	}
	ofstream ofs;
	ofs.open("ProblemOne.txt", ofstream::out);
	ofs << "Number of threads created: " << numThreads + 1 << endl << endl;
	ofs << "Expected number of steps: " << fixed << setprecision(6) << (double) totalSteps / (runPerThread*numThreads) << endl << endl;
	ofs << "Total number of runs needed for solution convergence: " << numThreads * runPerThread << endl << endl;
}
