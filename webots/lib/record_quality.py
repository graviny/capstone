"""Sampled record mismatch episodes, for evaluation only (never a WES input)."""


class RecordQuality:
    def __init__(self):
        self.open = {}
        self.closed = []
        self.area = 0.0
        self.last_t = 0.0
        self.last_count = 0

    def sample(self, t, wrong, busy):
        self.area += self.last_count * (t - self.last_t)
        # Busy slots are excluded from the count, but becoming busy is not proof
        # that a previously observed mismatch has actually been corrected.
        for sid in list(self.open):
            if sid not in wrong:
                self.closed.append(t - self.open.pop(sid))
        settled = wrong - busy
        for sid in settled:
            self.open.setdefault(sid, t)
        self.last_t, self.last_count = t, len(settled)
        return len(settled)

    def summary(self, t):
        area = self.area + self.last_count * (t - self.last_t)
        return {
            "metric_version": "sampled-settled-v2",
            "settled_mismatch_auc_slot_s": area,
            "settled_mismatch_mean": round(area / t, 3) if t else 0.0,
            "corrected": len(self.closed),
            "mean_time_to_correct_s": round(sum(self.closed) / len(self.closed), 1) if self.closed else None,
            "open_at_end": len(self.open),
            "mean_open_age_s": round(sum(t - v for v in self.open.values()) / len(self.open), 1) if self.open else None,
        }
