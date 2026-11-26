"""Durable command identity/results; not a queue or automatic task scheduler."""
import json
from pathlib import Path
import sqlite3


class CommandJournal:
    def __init__(self, path=":memory:"):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path))
        self.db.execute("CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, fingerprint TEXT, event TEXT)")
        self.db.commit()

    @staticmethod
    def fingerprint(message):
        return json.dumps({key: message.get(key) for key in (
            "type", "robot_id", "frame_id", "map_version", "goal")}, sort_keys=True, separators=(",", ":"))

    def get(self, command_id):
        row = self.db.execute("SELECT fingerprint,event FROM commands WHERE id=?", (command_id,)).fetchone()
        return (row[0], json.loads(row[1])) if row else None

    def put(self, command_id, fingerprint, event):
        self.db.execute("INSERT INTO commands VALUES (?,?,?) ON CONFLICT(id) DO UPDATE SET event=excluded.event",
                        (command_id, fingerprint, json.dumps(event, allow_nan=False)))
        self.db.commit()

    def events(self):
        return [(key, fingerprint, json.loads(event)) for key, fingerprint, event in
                self.db.execute("SELECT id,fingerprint,event FROM commands")]

    def close(self):
        self.db.close()
