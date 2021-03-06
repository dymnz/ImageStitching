#include "RouteHandler.h"


using namespace std;

//Find connecting route
void RouteHandler::findConnectingRoute(MatchTracker &matchTracker)
{
	int imageCount = matchTracker.getSize();
	//int pivotIndex = 0;
	const float fpThreshold = 0.3;
	const int fpBottomLimit = 10;
	vector <int> route;
	vector <pair<int, int> > stack;
	int next, index, current;
	for (int i = 0; i < imageCount; i++)
	{
		stack.clear();
		route.clear();
		if (i == pivotIndex) continue;
		route.push_back(i);
		index = 1;
		int maxMatch = 0;
		vector<int> currentLine = matchTracker.getPairNum(i);
		for (int r = 0; r < imageCount; r++)
		{
			if (r == i) continue;
			if (currentLine[r]>maxMatch)
				maxMatch = currentLine[r];
		}
		for (int r = 0; r < imageCount; r++)
		{
			if (r == i) continue;
			if (currentLine[r] < maxMatch*fpThreshold ||
				currentLine[r] < fpBottomLimit)
				continue;
			stack.push_back(pair<int, int>(r, index));
		}

		while (!stack.empty())
		{
			next = stack.back().first;
			route.push_back(next);
			stack.pop_back();

			if (next == pivotIndex)
			{
				matchTracker.assignRoute(i, route);
				if (stack.size() == 0)
					current = 0;
				else
					current = stack.back().second;
				while (route.size() > current)
					route.pop_back();
				index = current;
				continue;
			}
			bool isPushed = false;
			index++;
			vector<int> currentLine = matchTracker.getPairNum(next);
			for (int r = 0; r < imageCount; r++)
			{
				if (r == i) continue;
				if (currentLine[r]>maxMatch)
					maxMatch = currentLine[r];
			}
			for (int r = 0; r < imageCount; r++)
			{
				if (r == next) continue;

				if (currentLine[r] < maxMatch*fpThreshold ||
					currentLine[r] < fpBottomLimit)
					continue;

				bool duplicate = false;

				for (int p = 0; p < route.size(); p++)
				if (route[p] == r) duplicate = true;
				if (!duplicate)
				{
					stack.push_back(pair<int, int>(r, index));
					isPushed = true;
				}

			}
			if (!isPushed){
				if (stack.size() == 0)
					current = 0;
				else
					current = stack.back().second;
				while (route.size() > current)
					route.pop_back();

				index--;

			}
		}
	}
	countRoutesWeight(matchTracker);
	printf("Before:\n");
	printRoutes(matchTracker);
	removeRedundantRoutes(matchTracker);
	printf("After:\n");
	printRoutes(matchTracker);
}

void RouteHandler::removeRedundantRoutes(MatchTracker &matchTracker)
{
	int imageCount = matchTracker.getSize();
	for (int i = 0; i < imageCount; i++)
	{
		if (i == pivotIndex) continue;
		int maxAvgWeight = 0, maxAvgIndex = -1;
		Route &currentRoutes = matchTracker.getRoute(i);
		int routeCount = currentRoutes.route.size();
		for (int p = 0; p < routeCount; p++)
		{
			if (maxAvgWeight < currentRoutes.routeWeightAvg[p])
			{
				maxAvgIndex = p;
				maxAvgWeight = currentRoutes.routeWeightAvg[p];
			}
			else if (maxAvgWeight == currentRoutes.routeWeightAvg[p])
			{
				if (maxAvgIndex == -1)
					maxAvgIndex = p;
				else
					maxAvgIndex =
						currentRoutes.route[p].size() > currentRoutes.route[maxAvgIndex].size() ?
					maxAvgIndex : p;
				maxAvgWeight = currentRoutes.routeWeightAvg[maxAvgIndex];
			}
		}
		if (maxAvgIndex == -1) continue;
		vector<int> route = currentRoutes.route[maxAvgIndex];
		currentRoutes.route.clear();
		currentRoutes.route.push_back(route);
		currentRoutes.routeWeightAvg.clear();
		currentRoutes.routeWeightAvg.push_back(maxAvgWeight);
	}
}
void RouteHandler::countRoutesWeight(MatchTracker &matchTracker)
{
	int imageCount = matchTracker.getSize();
	for (int i = 0; i < imageCount; i++)
	{
		if (i == pivotIndex) continue;
		Route &currentRoutes = matchTracker.getRoute(i);
		int routeCount = currentRoutes.route.size();
		for (int r = 0; r < routeCount; r++)
		{
			int routeLength = currentRoutes.route[r].size();
			int weight = 0;
			for (int p = 0; p < routeLength; p++)
			{
				int x = currentRoutes.route[r][p];
				weight += matchTracker.getPairNum(i, x);
			}
			currentRoutes.routeWeight.push_back(weight);
			currentRoutes.routeWeightAvg.push_back(weight / routeLength);
		}
	}

}

void RouteHandler::printRoutes(MatchTracker &matchTracker)
{

	int imageCount = matchTracker.getSize();
	for (int i = 0; i < imageCount; i++)
	{
		if (i == pivotIndex) continue;
		Route &currentRoutes = matchTracker.getRoute(i);
		int routeCount = currentRoutes.route.size();
		if (routeCount == 0) printf("%d has no route\n", i);
		for (int r = 0; r < routeCount; r++)
		{
			int routeLength = currentRoutes.route[r].size();
			int weight = currentRoutes.routeWeight[r];
			int weightAvg = currentRoutes.routeWeightAvg[r];
			for (int p = 0; p < routeLength; p++)
			{
				int x = currentRoutes.route[r][p];

				printf("%d", x);
				if (p != routeLength - 1)
					printf("->");
			}

			printf("\t\t\t--- %d %d\n", weight, weightAvg);
		}
	}

}