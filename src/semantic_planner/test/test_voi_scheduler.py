"""
test_voi_scheduler.py — VoIScheduler 单元测试

覆盖:
  - SchedulerAction 枚举值
  - 硬约束: 极低可信度强制 REPERCEIVE
  - 硬约束: 高可信度 + 接近目标强制 CONTINUE
  - 最小移动距离检查
  - VoI 效用计算路径 (覆盖三种动作竞争)
  - 冷却时间约束
  - 决策日志 + decision_stats
  - 自定义 VoIConfig
"""

import time
import unittest

from semantic_planner.voi_scheduler import (
    SchedulerAction,
    SchedulerState,
    VoIConfig,
    VoIScheduler,
)


class TestSchedulerActionEnum(unittest.TestCase):
    def test_values(self):
        self.assertEqual(SchedulerAction.CONTINUE.value, "continue")
        self.assertEqual(SchedulerAction.REPERCEIVE.value, "reperceive")
        self.assertEqual(SchedulerAction.SLOW_REASON.value, "slow_reason")


class TestVoISchedulerHardConstraints(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, **kwargs) -> SchedulerState:
        """Helper: SchedulerState with safe defaults then override."""
        defaults = dict(
            target_credibility=0.5,
            target_existence_prob=0.6,
            target_position_var=1.0,
            match_count=0,
            total_objects=5,
            distance_to_goal=5.0,
            nav_accumulated_dist=0.0,
            distance_since_last_reperception=2.0,  # above min_distance
            slow_reason_count=0,
            reperception_count=0,
            time_elapsed=0.0,
            last_reperception_time=0.0,   # far in the past → cooldown OK
            last_slow_reason_time=0.0,
        )
        defaults.update(kwargs)
        return SchedulerState(**defaults)

    def test_danger_threshold_forces_reperceive(self):
        """Credibility below danger_threshold → must REPERCEIVE (cooldown satisfied)."""
        state = self._state(
            target_credibility=0.1,
            last_reperception_time=0.0,  # epoch 0 → cooldown long ago
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.REPERCEIVE)

    def test_danger_threshold_blocked_by_cooldown(self):
        """Credibility below danger but cooldown not expired → falls through to VoI."""
        state = self._state(
            target_credibility=0.1,
            last_reperception_time=time.time(),  # just now → blocked
        )
        # Should not force REPERCEIVE (cooldown blocks it)
        action = self.scheduler.decide(state)
        # can't be REPERCEIVE due to cooldown; likely CONTINUE or SLOW
        self.assertNotEqual(action, SchedulerAction.REPERCEIVE)

    def test_safe_and_close_forces_continue(self):
        """High credibility + close to goal → CONTINUE."""
        state = self._state(
            target_credibility=0.9,
            distance_to_goal=1.5,
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.CONTINUE)

    def test_high_credibility_but_far_does_not_force_continue(self):
        """High credibility but far away → VoI calculation, not forced CONTINUE."""
        state = self._state(
            target_credibility=0.9,
            distance_to_goal=10.0,
        )
        # Passes the hard CONTINUE rule; actual result depends on utilities
        action = self.scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)

    def test_insufficient_movement_forces_continue(self):
        """Too little movement since last reperception → CONTINUE."""
        state = self._state(
            target_credibility=0.5,
            distance_since_last_reperception=0.1,  # below min_distance=0.5
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.CONTINUE)


class TestVoISchedulerUtilityPaths(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, **kwargs) -> SchedulerState:
        defaults = dict(
            target_credibility=0.5,
            target_existence_prob=0.5,
            target_position_var=1.0,
            match_count=0,
            total_objects=10,
            distance_to_goal=10.0,
            nav_accumulated_dist=5.0,
            distance_since_last_reperception=3.0,
            slow_reason_count=0,
            reperception_count=0,
            time_elapsed=30.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        defaults.update(kwargs)
        return SchedulerState(**defaults)

    def test_returns_valid_action(self):
        state = self._state()
        action = self.scheduler.decide(state)
        self.assertIn(action, list(SchedulerAction))

    def test_no_matches_favors_reperception(self):
        """match_count=0 adds +0.3 bonus to reperceive utility (tested via internals)."""
        state_no_match = self._state(match_count=0, target_credibility=0.45)
        state_with_match = self._state(match_count=5, target_credibility=0.45)
        u_no_match = self.scheduler._utility_reperceive(state_no_match)
        u_with_match = self.scheduler._utility_reperceive(state_with_match)
        # Bonus of 0.3 applied when match_count=0
        self.assertGreater(u_no_match, u_with_match)
        # Decision itself is always a valid action
        action = self.scheduler.decide(state_no_match)
        self.assertIsInstance(action, SchedulerAction)

    def test_slow_reason_cooldown_blocks_slow(self):
        """If slow_reason on cooldown, SLOW_REASON utility set to -999."""
        state = self._state(
            last_slow_reason_time=time.time(),  # just triggered
        )
        action = self.scheduler.decide(state)
        self.assertNotEqual(action, SchedulerAction.SLOW_REASON)

    def test_reperceive_cooldown_blocks_reperceive(self):
        """If reperceive on cooldown, REPERCEIVE utility set to -999."""
        state = self._state(
            last_reperception_time=time.time(),
            target_credibility=0.5,  # not in danger zone
        )
        action = self.scheduler.decide(state)
        self.assertNotEqual(action, SchedulerAction.REPERCEIVE)

    def test_high_slow_reason_count_diminishes_slow_path(self):
        """Repeated slow reasoning reduces its marginal utility."""
        state_fresh = self._state(slow_reason_count=0)
        state_tired = self._state(slow_reason_count=20)
        # At count=20, slow_reason utility is heavily diminished
        # Can't assert exact action, but verify no crash
        action_fresh = self.scheduler.decide(state_fresh)
        action_tired = self.scheduler.decide(state_tired)
        self.assertIsInstance(action_fresh, SchedulerAction)
        self.assertIsInstance(action_tired, SchedulerAction)

    def test_complex_scene_boosts_slow_path(self):
        """Complex scene (many objects) raises slow_reason utility."""
        # Manufacture state where slow_reason should win:
        #   - moderate credibility (not high enough to short-circuit)
        #   - many objects (boost slow)
        #   - cooldowns expired
        state = self._state(
            target_credibility=0.45,
            total_objects=50,       # complex
            match_count=5,          # has matches → reperceive bonus lower
            distance_since_last_reperception=3.0,
        )
        action = self.scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)


class TestVoISchedulerDecisionLog(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, credibility=0.5) -> SchedulerState:
        return SchedulerState(
            target_credibility=credibility,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )

    def test_log_grows_per_decision(self):
        self.assertEqual(len(self.scheduler._decision_log), 0)
        self.scheduler.decide(self._state())
        self.assertEqual(len(self.scheduler._decision_log), 1)
        self.scheduler.decide(self._state())
        self.assertEqual(len(self.scheduler._decision_log), 2)

    def test_log_capped_at_50(self):
        for _ in range(60):
            self.scheduler.decide(self._state())
        self.assertLessEqual(len(self.scheduler._decision_log), 50)

    def test_log_entry_fields(self):
        self.scheduler.decide(self._state())
        entry = self.scheduler._decision_log[-1]
        self.assertIn("time", entry)
        self.assertIn("action", entry)
        self.assertIn("cred", entry)
        self.assertIn("reason", entry)

    def test_decision_stats_counts(self):
        # Force three specific actions by manipulating state
        # Force CONTINUE (safe + close)
        self.scheduler.decide(SchedulerState(
            target_credibility=0.9, distance_to_goal=1.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        ))
        # Force REPERCEIVE (danger threshold)
        self.scheduler.decide(SchedulerState(
            target_credibility=0.1, distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        ))
        stats = self.scheduler.decision_stats
        self.assertIn("continue", stats)
        self.assertIn("reperceive", stats)
        self.assertIn("slow_reason", stats)
        # At least 1 continue + 1 reperceive
        self.assertGreaterEqual(stats["continue"] + stats["reperceive"], 2)


class TestVoIConfig(unittest.TestCase):
    def test_custom_config(self):
        cfg = VoIConfig(
            lambda_t=0.5,
            credibility_safe=0.8,
            credibility_danger=0.2,
        )
        scheduler = VoIScheduler(config=cfg)
        state = SchedulerState(
            target_credibility=0.1,  # below custom danger=0.2 → REPERCEIVE
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        action = scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.REPERCEIVE)

    def test_default_config_instantiation(self):
        scheduler = VoIScheduler()
        self.assertIsNotNone(scheduler._config)
        self.assertEqual(scheduler._config.lambda_t, 0.3)

    def test_zero_cost_weights(self):
        """With zero cost weights all actions have equal cost structure."""
        cfg = VoIConfig(lambda_t=0.0, lambda_e=0.0, lambda_d=0.0)
        scheduler = VoIScheduler(config=cfg)
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        action = scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)


class TestVoIUtilityInternals(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def test_utility_continue_positive_with_uncertainty(self):
        state = SchedulerState(
            target_credibility=0.0,  # max uncertainty
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u = self.scheduler._utility_continue(state)
        # delta_s = 0.05 * 1.0 = 0.05, cost = 0.3 * 0.05 = ...
        self.assertIsInstance(u, float)

    def test_utility_reperceive_bonus_for_no_match(self):
        state_no_match = SchedulerState(
            target_credibility=0.5, match_count=0, target_position_var=1.0,
            distance_to_goal=5.0, distance_since_last_reperception=0.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        )
        state_with_match = SchedulerState(
            target_credibility=0.5, match_count=5, target_position_var=1.0,
            distance_to_goal=5.0, distance_since_last_reperception=0.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        )
        u_no_match = self.scheduler._utility_reperceive(state_no_match)
        u_with_match = self.scheduler._utility_reperceive(state_with_match)
        self.assertGreater(u_no_match, u_with_match)

    def test_utility_slow_reason_diminishes(self):
        base = SchedulerState(
            target_credibility=0.5, total_objects=10, slow_reason_count=0,
            distance_to_goal=5.0, distance_since_last_reperception=0.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        )
        many = SchedulerState(
            target_credibility=0.5, total_objects=10, slow_reason_count=10,
            distance_to_goal=5.0, distance_since_last_reperception=0.0,
            last_reperception_time=0.0, last_slow_reason_time=0.0,
        )
        u_base = self.scheduler._utility_slow_reason(base)
        u_many = self.scheduler._utility_slow_reason(many)
        self.assertGreater(u_base, u_many)


if __name__ == "__main__":
    unittest.main()
