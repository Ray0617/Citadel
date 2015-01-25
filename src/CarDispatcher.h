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
//#define _DDEBUG
//#define FIND_NEAREST_PASSENGER
//#define FIND_BEST_PASSENGER
#define FIND_BEST_PAIR

struct PassengerInfo {
  PassengerRequest request;
  int status; // 0: wait; 1: moving or fulfill
  int car_id; // the car intending to pick up this passenger
  int wait_time; // the time when the car pick up passenger
};

struct CarInfo {
  CarCtl carctl;
  int arrival_time; // the arrival time of next node (not destination)
  int passenger_id; // the passenger the car try to pick up
  
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
    
    int center_x = 0;
    int center_y = 0;
    for (unsigned i = 0; i < intersections.size(); i++) {
      center_x += intersections[i].x_location;
      center_y += intersections[i].y_location;
    }
    center_x /= intersections.size();
    center_y /= intersections.size();
    
    int min_dist_square = INT_MAX;
    center = -1;
    for (unsigned i = 0; i < intersections.size(); i++) {
      int dx = intersections[i].x_location - center_x;
      int dy = intersections[i].y_location - center_y;
      double dist_square = dx * dx + dy * dy;
      if ( dist_square < min_dist_square) {
        min_dist_square = dist_square;
        center = i;
      }
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
#ifdef _DDEBUG
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

  static int turn;
  void onTurn(std::vector<CarCtl> & cars_at_intersections, 
              const std::vector<PassengerRequest> & passenger_requests) {
    turn++;
#ifdef _DEBUG    
    printf("turn %d:\n", turn);
    //if (turn > 15)
      //exit(0);
#endif
    
    for (unsigned i = 0; i < passenger_requests.size(); i++) {
      PassengerInfo info = {passenger_requests[i], 0, 0, 0};
      passenger_info[passenger_requests[i].passenger_id] = info;
    }
#ifdef _DEBUG
    // print all passenger info
    printf("passenger info:\n");
    for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
      printf("%d: (%d => %d) %d\n", it->second.request.passenger_id, 
        it->second.request.src_intersection_id, it->second.request.dst_intersection_id, it->second.status);
    }
#endif

    
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
      CarInfo info = {cars_at_intersections[i], turn, 0};
      car_info[cars_at_intersections[i].car_id] = info;
    }
#ifdef _DEBUG
    // print all car info
    printf("decision car info:\n");
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
      printf("%d %d %d\n", cars_at_intersections[i].car_id, cars_at_intersections[i].intersection_id, 
        cars_at_intersections[i].passenger_id);
    }
    printf("car info (before):\n");
    for (map<int, CarInfo>::iterator it = car_info.begin(); it != car_info.end(); it++) {
      printf("%d %d %d %d\n", it->second.carctl.car_id, it->second.carctl.intersection_id, 
        it->second.carctl.passenger_id, it->second.arrival_time);
    }
#endif
    
#ifdef FIND_NEAREST_PASSENGER
    findNearestPassenger(cars_at_intersections);
    return;
#else
#ifdef FIND_BEST_PASSENGER
    findBestPassenger(cars_at_intersections);
#endif
#ifdef FIND_BEST_PAIR
    findBestPair(cars_at_intersections);
#endif
#endif

#ifdef _DEBUG
    // print all car info
    printf("car info (after):\n");
    for (map<int, CarInfo>::iterator it = car_info.begin(); it != car_info.end(); it++) {
      printf("%d %d %d %d\n", it->second.carctl.car_id, it->second.carctl.intersection_id, 
        it->second.carctl.passenger_id, it->second.arrival_time);
    }
#endif

  }
  
  void findNearestPassenger(std::vector<CarCtl> & cars_at_intersections) {
    // for each idle car, decide if need to find the nearest passenger or move to place without cars
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
      if (cars_at_intersections[i].passenger_id >= 0) { // with passenger; just find a nearest route to dst
        cars_at_intersections[i].intersection_id = findNextNode(
          cars_at_intersections[i].intersection_id, 
          passenger_info[cars_at_intersections[i].passenger_id].request.dst_intersection_id);
      } else { // no passenger; try find a nearest passenger 
        int bestScore = INT_MAX;
        int bestPassenger = -1;
        for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
          if (it->second.status)
            continue;
          int dist = findDistance(cars_at_intersections[i].intersection_id, it->second.request.src_intersection_id);    
          if (dist < bestScore) { // might be optimized by checking if passenger's dst has another passenger 
            bestScore = dist;
            bestPassenger = it->first;
          }
        }
        if (bestPassenger >= 0) {
          if (bestScore == 0) { // just pickup the passenger, and toward to dst
            cars_at_intersections[i].passenger_id = passenger_info[bestPassenger].request.passenger_id;
            passenger_info[bestPassenger].status = 1;
            cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
              passenger_info[bestPassenger].request.dst_intersection_id);
          } else {  // toward to the passenger
            cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
              passenger_info[bestPassenger].request.src_intersection_id);
          }
        } else { // no passenger at all, do nothing
          // might be optimized by checking the distribution of cars
          continue;
        }
      }
    }
  }

  void findBestPassenger(std::vector<CarCtl> & cars_at_intersections) {
    // for each idle car, decide if need to find the best passenger or move to place without cars
    // best passenger means could be completely served earliest
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
      if (cars_at_intersections[i].passenger_id >= 0) { // with passenger; just find a nearest route to dst
        cars_at_intersections[i].intersection_id = findNextNode(
          cars_at_intersections[i].intersection_id, 
          passenger_info[cars_at_intersections[i].passenger_id].request.dst_intersection_id);
      } else { // no passenger; try find the best passenger 
        int bestScore = INT_MAX;
        int bestPassenger = -1;
        for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
          if (it->second.status)
            continue;
          int score = findDistance(cars_at_intersections[i].intersection_id, it->second.request.src_intersection_id) +
            findDistance(it->second.request.src_intersection_id, it->second.request.dst_intersection_id);
          if (score < bestScore) { // might be optimized by checking if passenger's dst has another passenger 
            bestScore = score;
            bestPassenger = it->first;
          }
        }
#ifdef _DEBUG
        printf("best passenger %d with score %d\n", bestPassenger, bestScore);
#endif        
        if (bestPassenger >= 0) {
          if (passenger_info[bestPassenger].request.src_intersection_id == cars_at_intersections[i].intersection_id) { // just pickup the passenger, and toward to dst
            cars_at_intersections[i].passenger_id = passenger_info[bestPassenger].request.passenger_id;
            passenger_info[bestPassenger].status = 1;
            cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
              passenger_info[bestPassenger].request.dst_intersection_id);
          } else {  // toward to the passenger
            cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
              passenger_info[bestPassenger].request.src_intersection_id);
          }
        } else { // no passenger at all, do nothing
          // might be optimized by checking the distribution of cars
          continue;
        }
      }
    }
  }

  int findScore(int car_id, int passenger_id) {
    if (car_info[car_id].carctl.passenger_id >= 0) {
      return (car_info[car_id].arrival_time - turn) +
        findDistance(car_info[car_id].carctl.intersection_id, 
          passenger_info[car_info[car_id].carctl.passenger_id].request.dst_intersection_id) +
        findDistance(passenger_info[car_info[car_id].carctl.passenger_id].request.dst_intersection_id, 
          passenger_info[passenger_id].request.src_intersection_id);
    } else {
      return (car_info[car_id].arrival_time - turn) +
        findDistance(car_info[car_id].carctl.intersection_id, passenger_info[passenger_id].request.src_intersection_id);
    }
  }
  
  void findBestPair(std::vector<CarCtl> & cars_at_intersections) {
    if (cars_at_intersections.empty())
      return;
    bool all_busy = true;
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
      if (cars_at_intersections[i].passenger_id < 0) {
        all_busy = false;
        break;
      }
    }
    
    map<int, map<int, int> > scores;
    multimap<int, pair<int, int> > sorted;
    if (!all_busy) {
      // update passenger_info[i].car_id and car_info[i].passenger_id
      for (map<int, CarInfo>::iterator it = car_info.begin(); it != car_info.end(); it++) {
        it->second.passenger_id = -1;
        for (map<int, PassengerInfo>::iterator ij = passenger_info.begin(); ij != passenger_info.end(); ij++) {
          if (ij->second.status)
            continue;
            int score = 0;
            if (it->second.carctl.passenger_id >= 0) {
            score = (it->second.arrival_time - turn) +
              findDistance(it->second.carctl.intersection_id, passenger_info[it->second.carctl.passenger_id].request.dst_intersection_id) +
              findDistance(passenger_info[it->second.carctl.passenger_id].request.dst_intersection_id, 
                ij->second.request.src_intersection_id);
          } else {
            score = (it->second.arrival_time - turn) +
              findDistance(it->second.carctl.intersection_id, ij->second.request.src_intersection_id);
          }
          // tuning the magic number: try to differential long trip passengers from short trip ones.
          double magic_number = 0.6;    //0.2: 145948; 0.4: 145956; 0.6: 145961; 0.8: 145958; 1: 145951
          score += magic_number * findDistance(ij->second.request.src_intersection_id, ij->second.request.dst_intersection_id);
          scores[it->second.carctl.car_id][ij->second.request.passenger_id] = score;
          sorted.insert(make_pair(score, make_pair(it->second.carctl.car_id, ij->second.request.passenger_id)));
        }
      }

      for (map<int, PassengerInfo>::iterator it = passenger_info.begin(); it != passenger_info.end(); it++) {
        it->second.car_id = -1;
      }
    
      // priority: minimum score first
      map<int, vector<int> > candidates;
      for (multimap<int, pair<int, int> >::iterator it = sorted.begin(); it != sorted.end(); it++) {
        candidates[it->second.second].push_back(it->second.first);
        if (car_info[it->second.first].passenger_id >= 0 || passenger_info[it->second.second].car_id >= 0)
          continue;
        passenger_info[it->second.second].car_id = it->second.first;
        car_info[it->second.first].passenger_id = it->second.second;
      }
      map<int, int> cid;
      int id = 0;
      for (map<int, vector<int> >::iterator it = candidates.begin(); it != candidates.end(); it++) {
        cid[id] = it->first;
        id++;
      }
      
      // bruteforce (due to the approach of deadline)
      if (0) {
          int min_score = INT_MAX;
          vector<int> arg_select;
          vector<int> select;
          while (true) {
            if (select.size() == candidates.size()) {
              // test the score
              int score = 0;
              for (unsigned i = 0; i < select.size(); i++) {
                score += findScore(select[i], cid[i]);
              }
              if (score < min_score) {
                min_score = score;
                arg_select = select;
              }
              
            }
          }
      }
    }
    
    
    for (unsigned i = 0; i < cars_at_intersections.size(); i++) {
        if (cars_at_intersections[i].passenger_id >= 0) { // with passenger; just find a nearest route to dst
        cars_at_intersections[i].intersection_id = findNextNode(
          cars_at_intersections[i].intersection_id, 
          passenger_info[cars_at_intersections[i].passenger_id].request.dst_intersection_id);
      } else if (car_info[cars_at_intersections[i].car_id].passenger_id >= 0) { // no passenger; try find the best passenger 
        int bestPassenger = car_info[cars_at_intersections[i].car_id].passenger_id;
#ifdef _DEBUG
        printf("best passenger %d\n", bestPassenger);
#endif
        if (passenger_info[bestPassenger].request.src_intersection_id == cars_at_intersections[i].intersection_id) { 
          // just pickup the passenger, and toward to dst
          cars_at_intersections[i].passenger_id = passenger_info[bestPassenger].request.passenger_id;
          passenger_info[bestPassenger].status = 1;
          int prev_intersection_id = cars_at_intersections[i].intersection_id;
          cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
          passenger_info[bestPassenger].request.dst_intersection_id);
          car_info[cars_at_intersections[i].car_id].carctl = cars_at_intersections[i];
          car_info[cars_at_intersections[i].car_id].arrival_time = turn + 
            findDistance(prev_intersection_id, cars_at_intersections[i].intersection_id);
        } else {  // toward to the passenger
          cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, 
          passenger_info[bestPassenger].request.src_intersection_id);
          car_info[cars_at_intersections[i].car_id].carctl = cars_at_intersections[i];
        }
      } else { // no passenger at all, do nothing
        // might be optimized by checking the distribution of cars
        //move idle car toward center
        if (0) {
          cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, center);
          car_info[cars_at_intersections[i].car_id].carctl = cars_at_intersections[i];
        }
        continue;
        
        // fail attempt...try to move idle car toward farest point, score drop fr 145944.5 to 144007.9
        if (0) {
          int farest_min_score = INT_MAX;
          int arg_farest = -1;
          for (unsigned j = 0; j < intersections.size(); j++) {
            int min_score = INT_MAX;
            for (map<int, CarInfo>::iterator it = car_info.begin(); it != car_info.end(); it++) {
              int score = it->second.arrival_time - turn;
                if (it->second.carctl.passenger_id >= 0) {
                  score += findDistance(it->second.carctl.intersection_id, 
                    passenger_info[it->second.carctl.passenger_id].request.dst_intersection_id);
                  score += findDistance(passenger_info[it->second.carctl.passenger_id].request.dst_intersection_id, j);
                } else {
                  score += findDistance(it->second.carctl.intersection_id, j);
                }
                if (score < min_score) {
                  min_score = score;
                }
            }
            if (min_score < farest_min_score) {
              farest_min_score = min_score;
                arg_farest = j;
            }
          }
          // toward the idle car to the farest position
          cars_at_intersections[i].intersection_id = findNextNode(cars_at_intersections[i].intersection_id, arg_farest);
          car_info[cars_at_intersections[i].car_id].carctl = cars_at_intersections[i];
        }
      }
    }
  }
private:
  vector<IntersectionInfo> intersections;
  vector<RoadInfo> roads;
  map<int, PassengerInfo> passenger_info;
  vector<vector<int> > shortest_node;
  vector<vector<int> > distance;
  vector<vector<RoadInfo> > roads_in_intersection;
  map<int, CarInfo> car_info;
  int center;
};

int CarDispatcher::turn = 0;

#endif
