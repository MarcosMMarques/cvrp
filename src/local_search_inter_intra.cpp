/**
 * @file local_search_inter_intra.cpp
 * @author vss2sn
 * @brief Contains the LocalSearchInterIntraSolution class (Local search extends
 * to all vehicles)
 */

#include "cvrp/local_search_inter_intra.hpp"

#include <iostream>
#include <omp.h>
#include <limits>
#include <numeric>
#include <thread>

constexpr double margin_of_error = 0.00001;

LocalSearchInterIntraSolution::LocalSearchInterIntraSolution(
    const std::vector<Node> &nodes, const std::vector<Vehicle> &vehicles,
    const std::vector<std::vector<double>> &distanceMatrix)
    : Solution(nodes, vehicles, distanceMatrix) {
  CreateInitialSolution();
}

LocalSearchInterIntraSolution::LocalSearchInterIntraSolution(const Problem &p)
    : Solution(p.nodes_, p.vehicles_, p.distanceMatrix_) {
  CreateInitialSolution();
}

LocalSearchInterIntraSolution::LocalSearchInterIntraSolution(const Solution &s)
    : Solution(s) {
  if (!s.CheckSolutionValid()) {
    std::cout << "The input solution is invalid. Exiting." << '\n';
    exit(0);
  }
}

void LocalSearchInterIntraSolution::Solve() {
  while (true) {
    int best_c_global = -1;
    int best_r_global = -1;
    Vehicle* v_temp_2_global = nullptr;
    Vehicle* v_temp_global = nullptr;
    double delta_global = std::numeric_limits<double>::max();

    #pragma omp parallel
    {
      int best_c_local = -1;
      int best_r_local = -1;
      Vehicle* v_temp_2_local = nullptr;
      Vehicle* v_temp_local = nullptr;
      double delta_local = std::numeric_limits<double>::max();

      #pragma omp for schedule(dynamic)
      for (size_t i = 0; i < vehicles_.size(); ++i) {
        auto& v = vehicles_[i];
        for (size_t cur = 1; cur < v.nodes_.size() - 1; cur++) {
          const int v_cur = v.nodes_[cur];
          const int v_prev = v.nodes_[cur - 1];
          const int v_next_c = v.nodes_[cur + 1];
          const double cost_reduction = distanceMatrix_[v_prev][v_next_c] -
                                        distanceMatrix_[v_prev][v_cur] -
                                        distanceMatrix_[v_cur][v_next_c];

          for (auto& v2 : vehicles_) {
            for (size_t rep = 0; rep < v2.nodes_.size() - 1; rep++) {
              const int v_rep = v2.nodes_[rep];
              const int v_next_r = v2.nodes_[rep + 1];

              if (v_rep != v_cur && (v.id_ != v2.id_ || v_rep != v_prev)) {
                const double cost_increase = distanceMatrix_[v_rep][v_cur] +
                                             distanceMatrix_[v_cur][v_next_r] -
                                             distanceMatrix_[v_rep][v_next_r];

                if (cost_increase + cost_reduction < delta_local &&
                    (v2.load_ - nodes_[v_cur].demand_ >= 0 || v.id_ == v2.id_)) {
                  delta_local = cost_increase + cost_reduction;
                  best_c_local = cur;
                  best_r_local = rep;
                  v_temp_2_local = &v2;
                  v_temp_local = &v;
                }
              }
            }
          }
        }
      }

      // Região crítica para atualizar os valores globais
      if (delta_local < delta_global) {
          #pragma omp critical
          {
            delta_global = delta_local;
            best_c_global = best_c_local;
            best_r_global = best_r_local;
            v_temp_2_global = v_temp_2_local;
            v_temp_global = v_temp_local;
          }
      }
    }

    // Condições de saída do loop
    if (delta_global > -margin_of_error || v_temp_global == nullptr || v_temp_2_global == nullptr) {
      break;
    }

    // Atualização da solução
    int val_best_c = *(v_temp_global->nodes_.begin() + best_c_global);
    v_temp_global->nodes_.erase(v_temp_global->nodes_.begin() + best_c_global);
    v_temp_global->CalculateCost(distanceMatrix_);
    if (v_temp_global->id_ == v_temp_2_global->id_ && best_c_global < best_r_global) {
      v_temp_2_global->nodes_.insert(std::next(v_temp_2_global->nodes_.begin(), best_r_global),
                                     val_best_c);
    } else {
      v_temp_2_global->nodes_.insert(std::next(v_temp_2_global->nodes_.begin(), best_r_global + 1),
                                     val_best_c);
    }
    v_temp_2_global->CalculateCost(distanceMatrix_);
    v_temp_global->load_ += nodes_[val_best_c].demand_;
    v_temp_2_global->load_ -= nodes_[val_best_c].demand_;
  }

  double cost = std::accumulate(
      std::begin(vehicles_), std::end(vehicles_), 0.0,
      [](const double sum, const Vehicle &v) { return sum + v.cost_; });
  std::cout << "Cost: " << cost << '\n';
  std::cout << "Solution valid: " << CheckSolutionValid() << '\n';
}



void LocalSearchInterIntraSolution::SolveSequential() {
  while (true) {
    int best_c_global = -1;
    int best_r_global = -1;
    Vehicle* v_temp_2_global = nullptr;
    Vehicle* v_temp_global = nullptr;
    double delta_global = std::numeric_limits<double>::max();

    {
      int best_c_local = -1;
      int best_r_local = -1;
      Vehicle* v_temp_2_local = nullptr;
      Vehicle* v_temp_local = nullptr;
      double delta_local = std::numeric_limits<double>::max();


      for (size_t i = 0; i < vehicles_.size(); ++i) {
        auto& v = vehicles_[i];
        for (size_t cur = 1; cur < v.nodes_.size() - 1; cur++) {
          const int v_cur = v.nodes_[cur];
          const int v_prev = v.nodes_[cur - 1];
          const int v_next_c = v.nodes_[cur + 1];
          const double cost_reduction = distanceMatrix_[v_prev][v_next_c] -
                                        distanceMatrix_[v_prev][v_cur] -
                                        distanceMatrix_[v_cur][v_next_c];

          for (auto& v2 : vehicles_) {
            for (size_t rep = 0; rep < v2.nodes_.size() - 1; rep++) {
              const int v_rep = v2.nodes_[rep];
              const int v_next_r = v2.nodes_[rep + 1];

              if (v_rep != v_cur && (v.id_ != v2.id_ || v_rep != v_prev)) {
                const double cost_increase = distanceMatrix_[v_rep][v_cur] +
                                             distanceMatrix_[v_cur][v_next_r] -
                                             distanceMatrix_[v_rep][v_next_r];

                if (cost_increase + cost_reduction < delta_local &&
                    (v2.load_ - nodes_[v_cur].demand_ >= 0 || v.id_ == v2.id_)) {
                  delta_local = cost_increase + cost_reduction;
                  best_c_local = cur;
                  best_r_local = rep;
                  v_temp_2_local = &v2;
                  v_temp_local = &v;
                }
              }
            }
          }
        }
      }



      {
        if (delta_local < delta_global) {
          delta_global = delta_local;
          best_c_global = best_c_local;
          best_r_global = best_r_local;
          v_temp_2_global = v_temp_2_local;
          v_temp_global = v_temp_local;
        }
      }
    }

    if (delta_global > -margin_of_error || v_temp_global == nullptr || v_temp_2_global == nullptr) {
      break;
    }

    int val_best_c = *(v_temp_global->nodes_.begin() + best_c_global);
    v_temp_global->nodes_.erase(v_temp_global->nodes_.begin() + best_c_global);
    v_temp_global->CalculateCostSequential(distanceMatrix_);
    if (v_temp_global->id_ == v_temp_2_global->id_ && best_c_global < best_r_global) {
      v_temp_2_global->nodes_.insert(std::next(v_temp_2_global->nodes_.begin(), best_r_global),
                                     val_best_c);
    } else {
      v_temp_2_global->nodes_.insert(std::next(v_temp_2_global->nodes_.begin(), best_r_global + 1),
                                     val_best_c);
    }
    v_temp_2_global->CalculateCostSequential(distanceMatrix_);
    v_temp_global->load_ += nodes_[val_best_c].demand_;
    v_temp_2_global->load_ -= nodes_[val_best_c].demand_;
  }

  double cost = std::accumulate(
      std::begin(vehicles_), std::end(vehicles_), 0.0,
      [](const double sum, const Vehicle &v) { return sum + v.cost_; });
  std::cout << "Cost: " << cost << '\n';
  // for (const auto &i : nodes_) {
  //   if (!i.is_routed_) {
  //     std::cout << "Unreached node: " << '\n';
  //     std::cout << i << '\n';
  //   }
  // }
  std::cout << "Solution valid: " << CheckSolutionValidSequential() << '\n';

}
