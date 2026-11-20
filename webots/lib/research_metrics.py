"""Evaluation only. No truth or future orders are supplied to a scheduler."""
from collections import defaultdict


class FlowMetrics:
    def __init__(self):
        self.events = []

    def observe(self, event):
        if event['type'] in ('ORDER_RELEASED', 'OUTBOUND_DEPARTED'):
            self.events.append(event.copy())

    def summary(self, horizon):
        outstanding, area, last = 0, 0., 0.
        released, shipped = 0, 0
        for ev in sorted(self.events, key=lambda e: (e['t'], e.get('event_id', ''))):
            t = min(horizon, ev['t'])
            area += outstanding * max(0, t - last)
            last = t
            if ev['type'] == 'ORDER_RELEASED':
                qty = sum(line['qty'] for line in ev['lines'])
                released += qty
                outstanding += qty
            else:
                # Shipment count here is physical tagged shipments, not a replacement
                # for simulator mis_shipped correctness checks.
                qty = sum(c.get('order') is not None for c in ev['cartons'])
                shipped += qty
                outstanding -= qty
            if outstanding < 0:
                raise ValueError('shipments exceed released demand')
        area += outstanding * max(0, horizon - last)
        return {'unit_backlog_auc_s': area, 'unit_backlog_mean': area / horizon if horizon else 0.,
                'unit_backlog_at_end': outstanding, 'tagged_shipped': shipped, 'released': released}


def order_metrics(orders, horizon):
    selected = [o for o in orders.values() if o['released_t'] <= horizon]
    if not selected:
        return {'orders_unfinished': 0, 'restricted_order_flow_mean_s': None, 'max_unfinished_order_age_s': None}
    unfinished = [o for o in selected if o['done_t'] is None]
    return {'orders_unfinished': len(unfinished),
            'restricted_order_flow_mean_s': sum((o['done_t'] if o['done_t'] is not None else horizon)
                                                - o['released_t'] for o in selected) / len(selected),
            'max_unfinished_order_age_s': max((horizon - o['released_t'] for o in unfinished), default=None)}


def task_metrics(tasks, horizon):
    groups = defaultdict(list)
    for task in tasks.values():
        groups[task['task_type']].append(task)
    out = {}
    for kind, rows in groups.items():
        never = [t for t in rows if t.get('first_assigned_t') is None and t['status'] != 'CANCELLED']
        served = [t['first_assigned_t'] - t['created_t'] for t in rows if t.get('first_assigned_t') is not None]
        out[kind] = {'created': len(rows), 'never_assigned_non_cancelled': len(never),
                     'max_never_assigned_age_s': max((horizon - t['created_t'] for t in never), default=None),
                     'max_first_assignment_wait_s': max(served, default=None),
                     'mean_first_assignment_wait_s': sum(served) / len(served) if served else None}
    return out
