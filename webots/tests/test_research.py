import json
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'lib'))
from fault_tape import FaultTape, apply_worker_attempt
from research_metrics import FlowMetrics, order_metrics, task_metrics
from sched_learned import Assign
from travel_time import TravelTime


class ResearchTests(unittest.TestCase):
    def cfg(self):
        return {'seed': 1, 'fault_seed': 100, 'episode': {'duration_s': 3600},
                'faults': {'worker_move_per_h': 80, 'worker_remove_per_h': 30}}

    def test_tape_is_independent_of_step_and_other_rng_consumption(self):
        a, b = FaultTape(self.cfg()), FaultTape(self.cfg())
        slow = [e for t in range(1, 3601) for e in a.due(t)]
        fast = [e for t in range(1, 7201) for e in b.due(t / 2)]
        self.assertEqual(slow, fast)
        self.assertEqual(a.sha256, b.sha256)
        self.assertGreater(len(slow), 50)

    def test_s3_independent_of_task_destination_and_unrelated_pickups(self):
        a, b = FaultTape(self.cfg()), FaultTape(self.cfg())
        b.s3('OTHER', 5, .5)
        self.assertEqual(a.s3('LPN', 10, 1)['u'], b.s3('LPN', 40, 1)['u'])
        self.assertNotEqual(a.s3('LPN', 10, 1)['u'], a.audit[0]['u'])

    def test_no_stock_attempt_is_retained(self):
        tape = FaultTape(self.cfg())
        e = tape.events[0]
        result = apply_worker_attempt(e, 100, {}, {}, {}, 3, None, None, None, tape)
        self.assertIsNone(result)
        self.assertEqual(tape.audit[0]['outcome'], 'skipped_no_stock')

    def test_backlog_exact_integral_and_zero_shipments(self):
        m = FlowMetrics()
        m.observe({'type': 'ORDER_RELEASED', 't': 10, 'lines': [{'qty': 2}]})
        m.observe({'type': 'OUTBOUND_DEPARTED', 't': 30, 'cartons': [{'order': 'o'}]})
        s = m.summary(50)
        self.assertEqual(s['unit_backlog_auc_s'], 60)
        self.assertEqual(s['unit_backlog_at_end'], 1)
        self.assertEqual(FlowMetrics().summary(50)['unit_backlog_auc_s'], 0)

    def test_unfinished_orders_are_not_dropped(self):
        s = order_metrics({'a': {'released_t': 0, 'done_t': 10},
                          'b': {'released_t': 20, 'done_t': None}}, 100)
        self.assertEqual(s['restricted_order_flow_mean_s'], 45)
        self.assertEqual(s['orders_unfinished'], 1)

    def test_pending_tasks_not_dropped_from_wait_report(self):
        rows = {'a': {'task_type': 'PICK', 'created_t': 10, 'status': 'PENDING'},
                'b': {'task_type': 'PICK', 'created_t': 0, 'first_assigned_t': 15, 'status': 'COMPLETED'}}
        s = task_metrics(rows, 100)['PICK']
        self.assertEqual(s['max_never_assigned_age_s'], 90)
        self.assertEqual(s['max_first_assignment_wait_s'], 15)

    def test_candidate_limit_can_exclude_verify_and_putaway(self):
        rows = [{'task_id': str(i), 'task_type': 'PICK', 'created_t': 0} for i in range(30)]
        rows += [{'task_id': 'pa', 'task_type': 'PUTAWAY', 'created_t': 0},
                 {'task_id': 'vf', 'task_type': 'VERIFY', 'created_t': 0}]
        s = SimpleNamespace(t=100, pending_tasks=lambda: rows)
        assignment = Assign({'robots': []})
        self.assertEqual(len(assignment.candidate_tasks(s)), 30)
        self.assertEqual(assignment.candidate_audit['excluded_task_steps'], {'PUTAWAY': 1, 'VERIFY': 1})
        assignment.candidate_limit = None
        self.assertEqual(len(assignment.candidate_tasks(s)), 32)

    def test_nominal_route_respects_wall_and_service_time(self):
        cfg = {'world': {'x_min': 0, 'x_max': 6, 'y_min': 0, 'y_max': 6}, 'racks': [],
               'obstacles': [{'x_min': 2.8, 'x_max': 3.2, 'y_min': 0, 'y_max': 4}],
               'slots': [{'id': 'dst', 'approach': [4., 1.]}],
               'nav': {'speed_mps': 1, 'act_s': 2}, 'perception': {'look_s': 3}}
        model = TravelTime(cfg)
        self.assertGreater(model.distance([2., 1.], [4., 1.]), 6)
        seconds = model.completion([2., 1.], {'task_type': 'VERIFY'}, (None, 'dst'))
        self.assertAlmostEqual(seconds, model.distance([2., 1.], [4., 1.]) + 3)


class CandidateWindowTests(unittest.TestCase):
    """Head-of-line blocking: infeasible tasks at the queue head must not hide feasible ones."""

    def make(self, window):
        a = Assign.__new__(Assign)
        a.window = window
        a.endpoints = lambda s, t, taken: (t['task_id'], 'dst') if t['feasible'] else None
        tasks = [{'task_id': f'x{i}', 'feasible': False, 'task_type': 'PICK', 'created_t': 0} for i in range(40)]
        tasks.append({'task_id': 'ok', 'feasible': True, 'task_type': 'PUTAWAY', 'created_t': 5})
        return a, SimpleNamespace(t=10, pending_tasks=lambda: tasks)

    def test_raw_window_hides_feasible_task_behind_infeasible_head(self):
        a, s = self.make('raw')
        self.assertNotIn('ok', [t['task_id'] for t in a.candidate_tasks(s, set())])

    def test_feasible_window_keeps_it_and_stays_bounded(self):
        a, s = self.make('feasible')
        self.assertEqual([t['task_id'] for t in a.candidate_tasks(s, set())], ['ok'])
        a.candidate_limit = 1
        self.assertEqual(len(a.candidate_tasks(s, set())), 1)


if __name__ == '__main__':
    unittest.main()


class EndpointConflictTests(unittest.TestCase):
    def test_verify_and_putaway_skip_slot_taken_this_step(self):
        a = Assign.__new__(Assign)
        verify = {'task_type': 'VERIFY', 'destination': 'r2n12', 'source': None}
        putaway = {'task_type': 'PUTAWAY', 'destination': 'r2n12', 'source': 'B1'}
        self.assertIsNone(a.endpoints(None, verify, {'r2n12'}))
        self.assertIsNone(a.endpoints(None, putaway, {'r2n12'}))
        self.assertEqual(a.endpoints(None, verify, {'B1'}), (None, 'r2n12'))
        self.assertEqual(a.endpoints(None, putaway, {'B1'}), ('B1', 'r2n12'))


class AdaptiveWeightTests(unittest.TestCase):
    def test_weights_switch_on_smoothed_queue_pressure(self):
        import math
        a = Assign.__new__(Assign)
        a.w_low, a.w_high, a.adapt_tau, a.adapt_tc, a._ema, a._pt = [1.0], [2.0], 3.5, 300.0, None, None
        a.w = a.w_low
        state = lambda t, n: SimpleNamespace(t=t, pending_tasks=lambda: [0] * n, robots={0: 0, 1: 1, 2: 2, 3: 3})
        a.update_load_weights(state(0, 8))                  # 2 per robot
        self.assertEqual(a.w, [1.0])
        a.update_load_weights(state(10, 80))                # spike: smoothing keeps low weights
        self.assertEqual(a.w, [1.0])
        for t in range(20, 1500, 10):                       # sustained 20 per robot
            a.update_load_weights(state(t, 80))
        self.assertEqual(a.w, [2.0])

