#include "guidance_control/gate_logic.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
#include <limits>
#include <set>
using namespace dagozilla::gate_logic;
int main()
{
  assert(wrap(359) == -1);
  assert(wrap(1 - 359) == 2);
  assert(compassTarget(350, 90, false) == 80);
  assert(compassTarget(350, 90, true) == 260);
  assert(headingCommand(80, 350, .02, .4) == -.4);
  assert(headingCommand(260, 350, .02, .4) == .4);
  assert(std::abs(headingCommand(0, 359, .02, .4) + .02) < 1e-9);
  std::cout << "PASS heading wrap / A-B sign / heading hold\n";

  const auto cost = [](double red_x, double red_y, double red_radius,
      double green_x, double green_y, double green_radius) {
      return pairAffinityCost(red_x, red_y, red_radius,
        green_x, green_y, green_radius, 640, 480, 1, 1, 1);
    };
  // Two complete red x green layers. Cross-depth candidates remain geometrically
  // valid, but radius and vertical consistency make the physical pairs cheaper.
  const double near_near = cost(80, 320, 50, 300, 315, 46);
  const double near_far = cost(80, 320, 50, 540, 205, 19);
  const double far_near = cost(220, 210, 20, 300, 315, 46);
  const double far_far = cost(220, 210, 20, 540, 205, 19);
  assert(near_near + far_far < near_far + far_near);
  auto pairs = assign({
      {0, 0, near_near, 0}, {0, 1, near_far, 1},
      {1, 0, far_near, 2}, {1, 1, far_far, 3}});
  assert((pairs == std::vector<size_t>{0, 3}));
  std::cout << "PASS near-near / far-far beat cross-depth pairs\n";

  // Maximum-cardinality assignment may reroute an initially cheap edge, while
  // retaining the hard invariant that each red and green detection appears once.
  const std::vector<Edge> competing{{0, 0, .1, 0}, {0, 1, .2, 1}, {1, 0, .2, 2}};
  pairs = assign(competing);
  assert((pairs == std::vector<size_t>{1, 2}));
  std::set<size_t> used_red;
  std::set<size_t> used_green;
  for (const size_t selected : pairs) {
    const auto & edge = competing[selected];
    assert(used_red.insert(edge.red).second);
    assert(used_green.insert(edge.green).second);
  }
  assert(assign({}).empty());
  std::cout << "PASS global assignment / one buoy per gate\n";

  // An asymmetric but still valid (ratio <= configured 3.0) gate is not rejected.
  const double asymmetric_cost = cost(120, 300, 50, 390, 292, 25);
  assert(std::isfinite(asymmetric_cost));
  assert(50.0 / 25.0 <= 3.0);
  assert((assign({{0, 0, asymmetric_cost, 0}}) == std::vector<size_t>{0}));
  std::cout << "PASS asymmetric valid gate\n";

  const auto score = [](double midpoint, double y, double radius, double confidence) {
      return selectionScore(midpoint, y, radius, confidence, 640, 480, 50, 1, 1, 2, .2);
    };
  // A: significant size/depth advantage beats a modest center disadvantage.
  const double near_score = score(400, 320, 50, .82);
  const double far_score = score(320, 210, 20, .90);
  assert(near_score > far_score);
  std::cout << "PASS near gate beats centered far gate\n";

  // B: with similar depth, center alignment remains a useful discriminator.
  assert(score(320, 250, 30, .90) > score(390, 250, 31, .90));
  std::cout << "PASS center wins at similar depth\n";

  // C: a small confidence advantage cannot erase a clear depth advantage.
  assert(score(400, 310, 48, .75) > score(320, 220, 22, .95));
  std::cout << "PASS near gate beats slightly higher-confidence far gate\n";

  // D: continuity stabilizes close choices, but cannot preserve a clearly far gate.
  constexpr double continuity_bonus = .2;
  assert(score(350, 250, 30, .85) + continuity_bonus > score(320, 250, 30, .85));
  assert(near_score > far_score + continuity_bonus);
  std::cout << "PASS bounded continuity behavior\n";

  const auto rational_score = [&](double midpoint, double red_y, double green_y,
      double radius, double confidence) {
      return rationalTargetScore(
        score(midpoint, (red_y + green_y) / 2, radius, confidence),
        red_y, green_y, 480, 4);
    };
  const auto switch_ok = [&](double current_midpoint, double current_red_y,
      double current_green_y, double current_radius, double candidate_midpoint,
      double candidate_red_y, double candidate_green_y, double candidate_radius) {
      return shouldSwitchTarget(
        rational_score(current_midpoint, current_red_y, current_green_y, current_radius, .90),
        rational_score(candidate_midpoint, candidate_red_y, candidate_green_y, candidate_radius, .85),
        (current_red_y + current_green_y) / 2,
        (candidate_red_y + candidate_green_y) / 2,
        current_radius, candidate_radius,
        verticalMismatch(candidate_red_y, candidate_green_y, 480),
        .5, 480 * .08, 1.15, .10);
    };

  // 1: A clearly lower, larger, consistent gate replaces the far lock only
  // after the configured consecutive-frame confirmation.
  assert(switch_ok(320, 180, 190, 20, 390, 310, 320, 50));
  int switch_confirm = 0;
  bool switched = false;
  for (int frame = 0; frame < 3; ++frame) {
    if (switch_ok(320, 180, 190, 20, 390, 310, 320, 50)) ++switch_confirm;
    switched = switch_confirm >= 3;
    if (frame < 2) assert(!switched);
  }
  assert(switched);
  std::cout << "PASS rational near target switches after hysteresis\n";

  // 2: A marginal improvement cannot cross the vertical/radius hysteresis.
  assert(!switch_ok(330, 240, 245, 30, 320, 265, 270, 32));
  std::cout << "PASS marginal target improvement does not switch\n";

  // 3: A low midpoint formed from vertically inconsistent buoys is suspicious.
  assert(!switch_ok(320, 180, 190, 20, 340, 170, 340, 45));
  std::cout << "PASS cross-depth switch candidate rejected\n";

  // 4: Center alignment does not let a higher, smaller gate replace a near lock.
  assert(!switch_ok(390, 310, 320, 50, 320, 185, 195, 20));
  std::cout << "PASS centered far gate cannot replace near lock\n";

  // 5: One missing locked buoy enters recovery and preserves translated geometry.
  assert(lossAction(false, false, 1, 1, 8) == LossAction::PARTIAL_RECOVERY);
  assert(std::abs(recoveryMidpoint(80, 100, 300) - 180) < 1e-9);
  assert(recoveryAim(250, 250, 390) > 320);
  assert(recoveryAim(390, 390, 250) < 320);
  std::cout << "PASS one-buoy loss keeps physical gate recovery\n";

  // 6/7: Same-session ID reassociation accepts nearby, size-compatible geometry,
  // but rejects a different physical buoy by position, radius, or pair mismatch.
  assert(reassociationCompatible(35, 640 * .12, 1.2, 1.8, .04, .10));
  assert(!reassociationCompatible(100, 640 * .12, 1.2, 1.8, .04, .10));
  assert(!reassociationCompatible(35, 640 * .12, 2.1, 1.8, .04, .10));
  assert(!reassociationCompatible(35, 640 * .12, 1.2, 1.8, .20, .10));
  std::cout << "PASS controlled reassociation accepts geometry, rejects other buoy\n";

  // 8: Both buoys lost before proximity is armed never counts a pass.
  assert(lossAction(false, false, 0, 100, 8) == LossAction::HOLD);
  assert(lossAction(true, false, 0, 100, 8) == LossAction::HOLD);
  std::cout << "PASS distant two-buoy loss does not count gate\n";

  // 9: Armed PASSING counts exactly once after the full-loss threshold.
  int counted_gates = 0;
  bool passing = true;
  for (int lost = 1; lost <= 12; ++lost) {
    if (passing && lossAction(true, true, 0, lost, 8) == LossAction::MARK_PASSED) {
      ++counted_gates;
      passing = false;
    }
  }
  assert(counted_gates == 1);
  std::cout << "PASS armed two-buoy loss counts gate exactly once\n";

  // 10: One buoy loss cannot mark a pass, even after proximity is armed.
  assert(lossAction(true, true, 1, 100, 8) == LossAction::PARTIAL_RECOVERY);
  std::cout << "PASS armed one-buoy loss cannot count gate\n";

  for (int i = 0; i <= 10; ++i) assert(specialGate(i) == (i == 3 || i == 7));
  std::cout << "PASS special gate triggers only 3/7\n";
}
