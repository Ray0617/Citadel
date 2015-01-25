#ifndef CAR_DISPATCHER_H_
#define CAR_DISPATCHER_H_
#include <map>
#include <set>
#include <vector>
#include <deque>
#include <stdio.h>
#include "TransportationTypes.h"

#include <algorithm>
#include <limits.h>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <assert.h>
using namespace std;

//#define _DEBUG

struct PassengerInfo {
  PassengerRequest request;
  int status; // 0: wait; 1: moving & fulfill
};

class CarDispatcher { 

public:

  CarDispatcher(std::vector<IntersectionInfo> _intersections, std::vector<RoadInfo> _roads)
  : intersections(_intersections)
  , roads(_roads)
  {
	shortest_node.resize(intersections.size());
	distance.resize(intersections.size());
    for (unsigned i = 0; i < intersections.size(); i++) {
  	  shortest_node[i].resize(intersections.size());
	  distance[i].resize(intersections.size());
      for (unsigned j = 0; j < intersections.size(); j++) {
	    shortest_node[i][j] = -1;
		distance[i][j] = INT_MAX;
	  }
	  assert((unsigned)intersections[i].intersection_id == i);
	}
	
	roads_in_intersection.resize(intersections.size());
	for (unsigned i = 0; i < roads.size(); i++) {
	  roads_in_intersection[roads[i].src_intersection_id].push_back(roads[i]);
	}	
  }

  double directDistance(int id1, int id2) {
    int dx = intersections[id1].x_location - intersections[id2].x_location;
    int dy = intersections[id1].y_location - intersections[id2].y_location;
    return sqrt(dx * dx + dy * dy);
  }
  
  void init(int from) {
    vector<bool> dirty(intersections.size());
    dirty[from] = true;
    distance[from][from] = 0;
	shortest_node[from][from] = 0;
    bool found = true;
    while (found) {
    found = false;
    for (unsigned i = 0; i < intersections.size(); i++) {
      if (dirty[i]) {
    	for (unsigned r = 0; r < roads_in_intersection[i].size(); r++) {
    	  if (distance[from][i] == INT_MAX || 
    		roads_in_intersection[i][r].weight + distance[from][i] < 
    		distance[from][roads_in_intersection[i][r].dst_intersection_id]) {
    		  distance[from][roads_in_intersection[i][r].dst_intersection_id] = 
    		    roads_in_intersection[i][r].weight + distance[from][i];
    		  shortest_node[from][roads_in_intersection[i][r].dst_intersection_id] = 
    		    roads_in_intersection[i][r].src_intersection_id;
    		  dirty[roads_in_intersection[i][r].dst_intersection_id] = true;
    		  found = true;
			}
    	  }
    	}
    	dirty[i] = false;
      }
    }
#ifdef _DEBUG
    printf("init %d\n", from);
    for (unsigned i = 0; i < intersections.size(); i++) {
	  printf("to %d dist %d thr node %d\n", i, distance[from][i], shortest_node[from][i]);
	}
#endif			  
  }
  
  int findNextNode(int from, int to) {
    if (shortest_node[from][from] != 0) {
	  init(from);
	}
	int prev = shortest_node[from][to];
	if (prev == from)
	  return to;
	return findNextNode(from, prev);
  }
  
  int findDistance(int src, int dst) {
    if (shortest_node[src][src] != 0) {
      init(src);
	}
	return distance[src][dst];
  }

  void onTurn(std::vector<CarCtl> & cars_at_intersections, 
              const std::vector<PassengerRequest> & passenger_requests) {
#ifdef _DEBUG
	static int turn = 0;
	printf("turn %d:\n", turn++);
#endif
	
	for (unsigned i = 0; i < passenger_requests.size(); i++) {
	  PassengerInfo info = {passenger_requests[i], 0};
	  passenger_info[passenger_requests[i].passenger_id] = info;
	}
#ifdef _DEBUG
    printf("passenger info:\n");
	for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
	  printf("%d: (%d => %d) %d\n", it->second.request.passenger_id, 
	    it->second.request.src_intersection_id, it->second.request.dst_intersection_id, it->second.status);
	}
	// print all passenger info
#endif

    // might be optimized by checking passenger and decide which car would serve
	
#ifdef _DEBUG
    printf("car info:\n");
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
	  printf("%d %d %d\n", cars_at_intersections[i].car_id, cars_at_intersections[i].intersection_id, 
	    cars_at_intersections[i].passenger_id);
	}
#endif	
	// for each idle car, decide if need to find nearest passenger or move to place without cars
	for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
	  if (cars_at_intersections[i].passenger_id >= 0) { // with passenger; just find a nearest route to dst
	    cars_at_intersections[i].intersection_id = findNextNode(
		  cars_at_intersections[i].intersection_id, 
		  passenger_info[cars_at_intersections[i].passenger_id].request.dst_intersection_id);
	  } else { // no passenger; try find a nearest passenger 
	    int nearestDistance = INT_MAX;
	    int nearestPassenger = -1;
	    for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
		  if (it->second.status)
		    continue;
	      int dist = findDistance(cars_at_intersections[i].intersection_id, it->second.request.src_intersection_id);
#ifdef _DEBUG
          printf("distance from passenger %d is %d\n", it->first, dist);
#endif		
		  if (dist < nearestDistance) { // might be optimized by checking if passenger's dst has another passenger 
		    nearestDistance = dist;
		    nearestPassenger = it->first;
		  }
	    }
#ifdef _DEBUG
        printf("nearest passenger %d at distance %d\n", nearestPassenger, nearestDistance);
#endif		
	    if (nearestPassenger >= 0) {
	      if (nearestDistance == 0) { // just pickup the passenger, and toward to dst
		    cars_at_intersections[i].passenger_id = passenger_info[nearestPassenger].request.passenger_id;
		    passenger_info[nearestPassenger].status = 1;
			cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
			  passenger_info[nearestPassenger].request.dst_intersection_id);
		  } else {  // toward to the passenger
			cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
			  passenger_info[nearestPassenger].request.src_intersection_id);
#ifdef _DEBUG
            printf("findNextNode(%d,%d)=%d\n", cars_at_intersections[i].intersection_id, passenger_info[nearestPassenger].request.src_intersection_id, cars_at_intersections[i].intersection_id);
#endif		
		  }
		} else { // no passenger at all, do nothing
		  // might be optimized by checking the distribution of cars
		  continue;
		}
	  }
	}
  }
  
private:
  vector<IntersectionInfo> intersections;
  vector<RoadInfo> roads;
  map<int, pair<CarCtl, int> > cars;	// time remain to get destination
  map<int, PassengerInfo> passenger_info;
  vector<vector<int> > shortest_node;
  vector<vector<int> > distance;
  vector<vector<RoadInfo> > roads_in_intersection;
};



#endif
