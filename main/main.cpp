#include <iostream>
#include <vector>
#include "cvrp/local_search_inter_intra.hpp"
#include "cvrp/utils.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

struct Point{
    public:
        int x_, y_, id_;

    explicit Point(const int x = 0, const int y = 0, const int id = 0)
        : x_(x), y_(y), id_(id) {}
};

void printPoints(const std::vector<Point>& points) {
    std::cout << "Points:" << std::endl;
    for (const auto& point : points) {
        std::cout << "ID: " << point.id_ << ", x: " << point.x_ << ", y: " << point.y_ << std::endl;
    }
}

void printNodes(const std::vector<Node>& nodes) {
    std::cout << "Nodes:" << std::endl;
    for (const auto& node : nodes) {
        std::cout << "ID: " << node.id_ << ", x: " << node.x_ << ", y: " << node.y_
                  << ", demand: " << node.demand_ << ", is_routed: " << (node.is_routed_ ? "Yes" : "No") << std::endl;
    }
}

void printVehicles(const std::vector<Vehicle>& vehicles) {
    std::cout << "Vehicles:" << std::endl;
    for (const auto& vehicle : vehicles) {
        std::cout << "Vehicle ID: " << vehicle.id_ << ", Load: " << vehicle.load_
                  << ", Capacity: " << vehicle.capacity_ << ", Cost: " << vehicle.cost_ << std::endl;
        std::cout << "Nodes visited: ";
        for (const auto& node_id : vehicle.nodes_) {
            std::cout << node_id << " ";
        }
        std::cout << std::endl;
    }
}

void printDistanceMatrix(const std::vector<std::vector<double>>& distanceMatrix) {
    std::cout << "Distance Matrix:" << std::endl;
    for (const auto& row : distanceMatrix) {
        // for (const auto& dist : row) {
        //     std::cout << std::fixed << std::setprecision(2) << dist << "\t";
        // }
        // std::cout << std::endl;
        std::cout << row.size() << std::endl;
    }
}


int main() {
  // constexpr int noc = 31;
  constexpr int noc = 1249;
  constexpr int demand_range = 24;
  // constexpr int nov = 5;
  constexpr int nov = 725;
  constexpr int capacity = 100;
  // constexpr int grid_range = 100;
  constexpr int grid_range = 12500;
  Problem p(noc, demand_range, nov, capacity, grid_range, "uniform");

    std::string filename = "./../input_large_3.txt";
    std::ifstream file(filename);
    std::string line;
    int v_capacity = 0;
    int node_q = 0;
    std::vector<Point> points;
    std::vector<Node> nodes;
    std::vector<Vehicle> vehicles;
    std::vector<std::vector<double>> distanceMatrix;
    int num_vehicles = 0;

    if (file.is_open()) {
        while (getline(file, line)) {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;

            if (keyword == "CAPACITY") {
                iss.ignore(2);  // Ignorar ":"
                iss >> v_capacity;
            } else if (keyword == "DIMENSION") {
                iss.ignore(2);
                iss >> node_q;
            } else if (keyword == "NODE_COORD_SECTION") {
                for (int i = 0; i < node_q; ++i) {
                    int id, x, y;
                    file >> id >> x >> y;
                    id--;
                    Point point(x, y, id);
                    points.push_back(point);
                }
            } else if (keyword == "DEMAND_SECTION") {
                std::vector<Node> aux_nodes;
                for (int i = 0; i < node_q; ++i) {
                    int id, demand;
                    file >> id >> demand;
                    id--;
                    Node new_node(points[id].x_, points[id].y_, points[id].id_, demand, false);
                    aux_nodes.push_back(new_node);
                }
                nodes = aux_nodes;
            } else if (keyword == "DEPOT_SECTION") {
                // nodes[0].is_routed_ = true;
            } else if (keyword == "EOF") {
                break;
            } else if(keyword == "NAME"){
                num_vehicles = std::stoi(line.substr(line.find("-k") + 2));
            }
        }
        file.close();

        // Inicializar veículos com base na capacidade lida e na dimensão
        for (int i = 0; i < num_vehicles; ++i) {
            Vehicle new_v(i, v_capacity, v_capacity);
            new_v.nodes_.push_back(0);
            vehicles.push_back(new_v);
        }

        std::vector<double> tmp(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i) {
          distanceMatrix.push_back(tmp);
        }
        for (size_t i = 0; i < nodes.size(); ++i) {
          for (size_t j = i; j < nodes.size(); ++j) {
            distanceMatrix[i][j] = sqrt(pow((nodes[i].x_ - nodes[j].x_), 2) +
                                         pow((nodes[i].y_ - nodes[j].y_), 2));
            distanceMatrix[j][i] = distanceMatrix[i][j];
          }
        }
    }

    // std::cout << "Vehicle Capacity: " << v_capacity << std::endl;
    // std::cout << "Number of Nodes: " << node_q << std::endl;
    // printPoints(points);
    // printNodes(nodes);
    // printVehicles(vehicles);
    std::cout << "Local Search (Within all vehicles): " << '\n';
    LocalSearchInterIntraSolution vrp_lsii(nodes, vehicles, distanceMatrix);
    LocalSearchInterIntraSolution vrp_lsii2(nodes, vehicles, distanceMatrix);
    // LocalSearchInterIntraSolution vrp_lsii2(p);

    auto start = std::chrono::high_resolution_clock::now();
    vrp_lsii.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    vrp_lsii2.SolveSequential();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    // vrp_lsii2.Solve();
    // printDistanceMatrix(vrp_lsii2.GetDistanceMatrix());
    // printDistanceMatrix(distanceMatrix);
    std::cout << '\n';

  return 0;
}
