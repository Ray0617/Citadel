#ifndef CAR_DISPATCHER_H_
#define CAR_DISPATCHER_H_
#include <map>
#include <set>
#include <vector>
#include <stdio.h>
#include "TransportationTypes.h"

#include <algorithm>
#include <limits.h>
#include <fstream>
#include <stdlib.h>
#include <string>
using namespace std;

class CarDispatcher { 

public:

  // debug
  void outputRoadInfo(const RoadInfo& info, ofstream& output) {
    output << "road: " << info.road_id << "\t";
    output << info.src_intersection_id << "\t";
    output << info.dst_intersection_id << "\t";
    output << info.weight << endl;
  }

  void outputPassengerRequest(const PassengerRequest& request, ofstream& output) {
    output << "req: " << request.passenger_id << "\t" << request.src_intersection_id << "\t" << request.dst_intersection_id << endl;
  }
  
  void outputIntersectionInfo(const IntersectionInfo& info, ofstream& output) {
    output << "intersect: " << info.intersection_id << "\t" << info.x_location << "\t" << info.y_location << endl;
  }
  
  void outputCarCtl(const CarCtl& car, ofstream& output) {
    output << "car: " << car.car_id << "\t" << car.intersection_id << "\t" << car.passenger_id << endl;
  }
  
  void outputRoads() {
    ofstream output("roads.txt");
	for (unsigned i = 0; i < roads.size(); i++) {
	  outputRoadInfo(roads[i], output);
	}
	output.close();
  }

  void outputIntersections() {
    ofstream output("intersections.txt");
	for (unsigned i = 0; i < intersections.size(); i++) {
	  outputIntersectionInfo(intersections[i], output);
	}
	output.close();
  }

  CarDispatcher(std::vector<IntersectionInfo> _intersections, std::vector<RoadInfo> _roads)
  : intersections(_intersections)
  , roads(_roads)
  {
  
    //outputRoads();
    //outputIntersections();
    
	// init shortest path by intersections and roads
	vector<vector<int> > distance;
    shortestPath.resize(intersections.size());
    distance.resize(intersections.size());
    for (unsigned i = 0; i < intersections.size(); i++) {
	  shortestPath[i].resize(intersections.size());
      distance[i].resize(intersections.size());
      for (unsigned j = 0; j < intersections.size(); j++) {
  	    distance[i][j] = INT_MAX;
	  }
	  distance[i][i] = 0;
	}
	vector<pair<int,int> > dirty;
    for (unsigned i = 0; i < roads.size(); i++) {
	  distance[roads[i].src_intersection_id][roads[i].dst_intersection_id] = roads[i].weight;
	  dirty.push_back(make_pair(roads[i].src_intersection_id, roads[i].dst_intersection_id));
	  shortestPath[roads[i].src_intersection_id][roads[i].dst_intersection_id].insert(roads[i].dst_intersection_id);
	}
	while (!dirty.empty()) {
	  pair<int,int> road = dirty.back();
	  dirty.pop_back();
      for (unsigned i = 0; i < intersections.size(); i++) {
	    if (i == (unsigned)road.first || distance[i][road.first] == INT_MAX)
		  continue;
	    int dist = distance[i][road.first] + distance[road.first][road.second];
	    if (dist < distance[i][road.second]) {
		  distance[i][road.second] = dist;
		  dirty.push_back(make_pair(i, road.second));
		  shortestPath[i][road.second].clear();
		  shortestPath[i][road.second].insert(road.first);
		} else if (dist == distance[i][road.second]) {
		  shortestPath[i][road.second].insert(road.first);
		}
	  }
	}
	
	if (0) { // test shortestPath
	  ofstream output("shortestPath.txt");
	  for (unsigned i = 0; i < intersections.size(); i++) {
	    for (unsigned j = 0; j < intersections.size(); j++) {
	      if (distance[i][j] == INT_MAX)
	  	  output << "---- ";
	      else {
	  	  output << distance[i][j];
	  	  output << "(";
	  	  for (set<int>::iterator it = shortestPath[i][j].begin(); it != shortestPath[i][j].end(); it++) {
	  	    output << *it;
	  	  }
	  	  output << ") ";
	  	}
	    }
	    output << endl;
	  }
	  output.close();
	}
	
    center_x = 0;
	center_y = 0;
    for (unsigned i = 0; i < intersections.size(); i++) {
		center_x += intersections[i].x_location;
		center_y += intersections[i].y_location;
	}
	center_x /= intersections.size();
	center_y /= intersections.size();

    ofstream output("debug.txt");	// clean debug file
	output.close();
  }

  void onTurn(std::vector<CarCtl> & cars_at_intersections, 
              const std::vector<PassengerRequest> & _passenger_requests) {
	
	// version 0. 
	//testSimpleMap(cars_at_intersections, _passenger_requests);
	//return;
	
	
	int size = passenger_requests.size();
	passenger_requests.resize(size + _passenger_requests.size());
	copy(_passenger_requests.begin(), _passenger_requests.end(), passenger_requests.begin() + size);	
	
	// for each idle car, decide if need to forward to center
	// version 1. find nearest passenger and go
	vector<int> load(intersections.size());
	for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
	  // check if any passenger at the same position
	  for (unsigned p = 0; p < passenger_requests.size(); p++) {
	    if (passenger_requests[p].src_intersection_id == cars_at_intersections[i].intersection_id) {
		  cars_at_intersections[i].passenger_id = passenger_requests[p].passenger_id;
		  set<int>& paths = shortestPath[passenger_requests[p].src_intersection_id][passenger_requests[p].dst_intersection_id];
		  int minLoad = INT_MAX;
		  int minNode = -1;
		  for (set<int>::iterator it = paths.begin(); it != paths.end(); it++) {
		    if (load[*it] < minLoad) {
			  minLoad = load[*it];
			  minNode = *it;
			}
		  }
		  cars_at_intersections[i].intersection_id = minNode;
		  load[minNode]++;
		  passenger_requests[p] = passenger_requests[passenger_requests.size()-1];
		  passenger_requests.pop_back();
		  break;
		}
      }
	}
  }
  void testSimpleMap(std::vector<CarCtl> & cars_at_intersections, 
              const std::vector<PassengerRequest> & passenger_requests) {
    static int turn = 0;
    ofstream output("debug.txt", ios::app);
	output << "turn " << turn++ << ": " << endl;
	for (unsigned int i = 0; i < cars_at_intersections.size(); i++)
	  outputCarCtl(cars_at_intersections[i], output);
	for (unsigned int i = 0; i < passenger_requests.size(); i++)
	  outputPassengerRequest(passenger_requests[i], output);
	output.close();

	if (turn == 1) {
	  cars_at_intersections[0].intersection_id = 0;
	} else if (turn == 680) {
	  cars_at_intersections[0].intersection_id = 2;
	  cars_at_intersections[0].passenger_id = 0;
	} else if (turn == 1360) {
	  cars_at_intersections[0].intersection_id = 1;
	  cars_at_intersections[0].passenger_id = 1;
	} else if (turn == 1880) {
	  cars_at_intersections[0].intersection_id = 3;
	  cars_at_intersections[0].passenger_id = 1;
	} else if (turn == 2104) {
	  cars_at_intersections[0].intersection_id = 1;
	  cars_at_intersections[0].passenger_id = 3;
	} else if (turn == 2328) {
	  cars_at_intersections[0].intersection_id = 0;
	} else if (turn == 3007) {
	  cars_at_intersections[0].intersection_id = 2;
	} else if (turn == 3687) {
	  cars_at_intersections[0].intersection_id = 1;
	  cars_at_intersections[0].passenger_id = 2;
	}
  }
  
private:
  vector<IntersectionInfo> intersections;
  vector<RoadInfo> roads;
  map<int, pair<CarCtl, int> > cars;	// time remain to get destination
  vector<PassengerRequest> passenger_requests;
  vector<vector<set<int> > > shortestPath;	//[src][dst][n]: might have multiple shorest path (n)
  int center_x;
  int center_y;
};



#endif
