"""Persistent semantic-object lifecycle storage."""

import math
import sqlite3
import time


class SemanticStore:
    def __init__(self, path: str, confirm_observations: int = 3,
                 association_radius_m: float = 0.35):
        self.connection = sqlite3.connect(path)
        self.connection.row_factory = sqlite3.Row
        self.confirm_observations = confirm_observations
        self.association_radius_m = association_radius_m
        self.connection.execute("""
            CREATE TABLE IF NOT EXISTS objects (
              id INTEGER PRIMARY KEY AUTOINCREMENT, class TEXT NOT NULL,
              x REAL NOT NULL, y REAL NOT NULL, status TEXT NOT NULL,
              confidence REAL NOT NULL, camera_confirmed INTEGER NOT NULL,
              lidar_confirmed INTEGER NOT NULL, observation_count INTEGER NOT NULL,
              first_seen REAL NOT NULL, last_seen REAL NOT NULL
            )
        """)
        self.connection.commit()

    def observe(self, label: str, x: float, y: float, confidence: float,
                lidar_confirmed: bool, stamp: float | None = None) -> dict:
        stamp = stamp if stamp is not None else time.time()
        rows = self.connection.execute(
            "SELECT * FROM objects WHERE class=? AND status IN ('candidate','confirmed')", (label,)
        ).fetchall()
        match = min(rows, key=lambda row: math.hypot(x-row["x"], y-row["y"]), default=None)
        if match is not None and math.hypot(x-match["x"], y-match["y"]) <= self.association_radius_m:
            count = match["observation_count"] + 1
            weight = min(0.5, 1.0 / count + (0.2 if lidar_confirmed else 0.0))
            status = "confirmed" if count >= self.confirm_observations else "candidate"
            self.connection.execute(
                "UPDATE objects SET x=?,y=?,status=?,confidence=?,lidar_confirmed=?,"
                "observation_count=?,last_seen=? WHERE id=?",
                ((1-weight)*match["x"]+weight*x, (1-weight)*match["y"]+weight*y, status,
                 max(match["confidence"], confidence),
                 int(bool(match["lidar_confirmed"]) or lidar_confirmed), count, stamp, match["id"]),
            )
            object_id = match["id"]
        else:
            cursor = self.connection.execute(
                "INSERT INTO objects(class,x,y,status,confidence,camera_confirmed,lidar_confirmed,"
                "observation_count,first_seen,last_seen) VALUES(?,?,?,?,?,?,?,?,?,?)",
                (label, x, y, "candidate", confidence, 1, int(lidar_confirmed), 1, stamp, stamp),
            )
            object_id = cursor.lastrowid
        self.connection.commit()
        return self.get(object_id)

    def expire_candidates(self, now: float | None = None, ttl_s: float = 30.0) -> None:
        now = now if now is not None else time.time()
        self.connection.execute("DELETE FROM objects WHERE status='candidate' AND last_seen < ?",
                                (now-ttl_s,))
        self.connection.commit()

    def expire_classes(self, labels: tuple[str, ...], now: float | None = None,
                       ttl_s: float = 5.0) -> None:
        now = now if now is not None else time.time()
        placeholders = ",".join("?" for _ in labels)
        self.connection.execute(
            f"DELETE FROM objects WHERE class IN ({placeholders}) AND last_seen < ?",
            (*labels, now-ttl_s),
        )
        self.connection.commit()

    def get(self, object_id: int) -> dict:
        return dict(self.connection.execute("SELECT * FROM objects WHERE id=?", (object_id,)).fetchone())

    def all(self, confirmed_only: bool = False) -> list[dict]:
        query = "SELECT * FROM objects" + (" WHERE status='confirmed'" if confirmed_only else "")
        return [dict(row) for row in self.connection.execute(query + " ORDER BY id")]

    def clear(self) -> None:
        """Delete all semantic objects in the current map session."""
        self.connection.execute("DELETE FROM objects")
        self.connection.execute("DELETE FROM sqlite_sequence WHERE name='objects'")
        self.connection.commit()

    def close(self) -> None:
        self.connection.close()
