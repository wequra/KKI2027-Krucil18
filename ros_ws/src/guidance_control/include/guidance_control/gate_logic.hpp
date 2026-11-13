#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace dagozilla::gate_logic
{
inline double wrap(double degrees)
{
  return degrees - 360.0 * std::floor((degrees + 180.0) / 360.0);
}
inline double compassTarget(double start, double angle, bool layout_a)
{
  const double value = wrap(start + (layout_a ? -angle : angle));
  return value < 0 ? value + 360 : value;
}
inline double headingCommand(double target, double current, double kp, double limit)
{
  return std::clamp(-kp * wrap(target - current), -limit, limit);
}
inline bool specialGate(int count) {return count == 3 || count == 7;}

inline double pairAffinityCost(
  double red_x, double red_y, double red_radius,
  double green_x, double green_y, double green_radius,
  double width, double height,
  double width_weight, double vertical_weight, double radius_weight)
{
  return width_weight * std::abs(red_x - green_x) / width +
         vertical_weight * std::abs(red_y - green_y) / height +
         radius_weight * std::abs(std::log(red_radius / green_radius));
}

// Minimum-cost maximum bipartite matching. It first pairs as many red/green
// detections as possible, then minimizes total geometric/depth affinity cost.
// Explicit detection indices guarantee that no buoy is used twice.
struct Edge {size_t red, green; double cost; size_t index;};
inline std::vector<size_t> assign(std::vector<Edge> edges)
{
  edges.erase(std::remove_if(edges.begin(), edges.end(), [](const Edge & edge) {
    return !std::isfinite(edge.cost) || edge.cost < 0.0;
  }), edges.end());
  if (edges.empty()) return {};

  std::stable_sort(edges.begin(), edges.end(), [](const Edge & a, const Edge & b) {
    if (a.red != b.red) return a.red < b.red;
    if (a.green != b.green) return a.green < b.green;
    if (a.cost != b.cost) return a.cost < b.cost;
    return a.index < b.index;
  });

  std::vector<size_t> reds;
  std::vector<size_t> greens;
  for (const auto & edge : edges) {
    reds.push_back(edge.red);
    greens.push_back(edge.green);
  }
  std::sort(reds.begin(), reds.end());
  reds.erase(std::unique(reds.begin(), reds.end()), reds.end());
  std::sort(greens.begin(), greens.end());
  greens.erase(std::unique(greens.begin(), greens.end()), greens.end());

  struct FlowEdge
  {
    int to;
    int reverse;
    int capacity;
    double cost;
    size_t result_index;
    bool assignment;
  };
  const int source = 0;
  const int red_begin = 1;
  const int green_begin = red_begin + static_cast<int>(reds.size());
  const int sink = green_begin + static_cast<int>(greens.size());
  std::vector<std::vector<FlowEdge>> graph(static_cast<size_t>(sink + 1));
  const auto add_edge = [&](int from, int to, double cost, size_t result_index,
      bool assignment) {
      const int forward_reverse = static_cast<int>(graph[to].size());
      const int reverse_reverse = static_cast<int>(graph[from].size());
      graph[from].push_back({to, forward_reverse, 1, cost, result_index, assignment});
      graph[to].push_back({from, reverse_reverse, 0, -cost, result_index, false});
    };
  for (size_t i = 0; i < reds.size(); ++i) {
    add_edge(source, red_begin + static_cast<int>(i), 0.0, 0, false);
  }
  for (size_t i = 0; i < greens.size(); ++i) {
    add_edge(green_begin + static_cast<int>(i), sink, 0.0, 0, false);
  }
  for (const auto & edge : edges) {
    const auto red = std::lower_bound(reds.begin(), reds.end(), edge.red) - reds.begin();
    const auto green = std::lower_bound(greens.begin(), greens.end(), edge.green) - greens.begin();
    add_edge(red_begin + static_cast<int>(red), green_begin + static_cast<int>(green),
      edge.cost, edge.index, true);
  }

  while (true) {
    std::vector<double> distance(graph.size(), std::numeric_limits<double>::infinity());
    std::vector<int> previous_node(graph.size(), -1);
    std::vector<int> previous_edge(graph.size(), -1);
    std::vector<bool> queued(graph.size(), false);
    std::deque<int> queue;
    distance[source] = 0.0;
    queue.push_back(source);
    queued[source] = true;
    while (!queue.empty()) {
      const int from = queue.front();
      queue.pop_front();
      queued[from] = false;
      for (size_t i = 0; i < graph[from].size(); ++i) {
        const auto & edge = graph[from][i];
        if (edge.capacity == 0 || distance[from] + edge.cost >= distance[edge.to] - 1e-12) {
          continue;
        }
        distance[edge.to] = distance[from] + edge.cost;
        previous_node[edge.to] = from;
        previous_edge[edge.to] = static_cast<int>(i);
        if (!queued[edge.to]) {
          queue.push_back(edge.to);
          queued[edge.to] = true;
        }
      }
    }
    if (previous_node[sink] < 0) break;
    for (int node = sink; node != source; node = previous_node[node]) {
      auto & edge = graph[previous_node[node]][previous_edge[node]];
      --edge.capacity;
      ++graph[node][edge.reverse].capacity;
    }
  }

  std::vector<size_t> result;
  for (size_t i = 0; i < reds.size(); ++i) {
    for (const auto & edge : graph[red_begin + static_cast<int>(i)]) {
      if (edge.assignment && edge.capacity == 0) result.push_back(edge.result_index);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}
inline double selectionScore(double midpoint, double y, double mean_radius,
  double confidence, double width, double height, double size_reference_radius,
  double center_weight, double vertical_weight, double size_weight, double confidence_weight)
{
  return center_weight * (1.0 - std::abs(midpoint - width / 2) / (width / 2)) +
         vertical_weight * y / height + size_weight * mean_radius / size_reference_radius +
         confidence_weight * confidence;
}

inline double verticalMismatch(double red_y, double green_y, double height)
{
  return std::abs(red_y - green_y) / height;
}

inline double rationalTargetScore(
  double selection_score, double red_y, double green_y, double height,
  double mismatch_weight)
{
  return selection_score - mismatch_weight * verticalMismatch(red_y, green_y, height);
}

inline bool shouldSwitchTarget(
  double current_score, double candidate_score,
  double current_mean_y, double candidate_mean_y,
  double current_mean_radius, double candidate_mean_radius,
  double candidate_vertical_mismatch,
  double score_margin, double min_mean_y_advantage,
  double min_radius_ratio, double max_vertical_mismatch)
{
  return candidate_vertical_mismatch <= max_vertical_mismatch &&
         candidate_mean_y - current_mean_y >= min_mean_y_advantage &&
         candidate_mean_radius >= current_mean_radius * min_radius_ratio &&
         candidate_score >= current_score + score_margin;
}

inline bool reassociationCompatible(
  double position_distance, double max_position_distance,
  double radius_ratio, double max_radius_ratio,
  double vertical_mismatch, double max_vertical_mismatch)
{
  return position_distance <= max_position_distance &&
         radius_ratio <= max_radius_ratio &&
         vertical_mismatch <= max_vertical_mismatch;
}

inline double recoveryMidpoint(
  double visible_x, double previous_visible_x, double previous_missing_x)
{
  const double estimated_missing_x = visible_x + previous_missing_x - previous_visible_x;
  return (visible_x + estimated_missing_x) / 2.0;
}
inline double recoveryAim(
  double visible_x, double previous_visible_x, double previous_missing_x)
{
  const double midpoint = recoveryMidpoint(visible_x, previous_visible_x, previous_missing_x);
  const double missing_x = visible_x + previous_missing_x - previous_visible_x;
  return midpoint + 0.5 * (missing_x - midpoint);
}

enum class LossAction {TRACK, PARTIAL_RECOVERY, HOLD, MARK_PASSED};
inline LossAction lossAction(
  bool passing, bool pass_armed, int visible_locked,
  int lost_frames, int lost_frames_for_pass)
{
  if (visible_locked == 2) return LossAction::TRACK;
  if (visible_locked == 1) return LossAction::PARTIAL_RECOVERY;
  if (passing && pass_armed && lost_frames >= lost_frames_for_pass) {
    return LossAction::MARK_PASSED;
  }
  return LossAction::HOLD;
}
}  // namespace dagozilla::gate_logic
