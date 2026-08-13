from rover_navigation.semantic_store import SemanticStore


def test_candidate_becomes_confirmed_and_keeps_id(tmp_path):
    store = SemanticStore(str(tmp_path / "objects.db"), confirm_observations=3)
    first = store.observe("chair", 1.0, 2.0, 0.8, False, 1.0)
    store.observe("chair", 1.1, 2.0, 0.9, True, 2.0)
    third = store.observe("chair", 1.05, 2.0, 0.9, True, 3.0)
    assert first["id"] == third["id"]
    assert third["status"] == "confirmed"
    assert third["lidar_confirmed"] == 1


def test_stale_candidate_is_deleted(tmp_path):
    store = SemanticStore(str(tmp_path / "objects.db"))
    store.observe("chair", 0, 0, 0.5, False, 1.0)
    store.expire_candidates(now=40.0, ttl_s=30.0)
    assert store.all() == []
