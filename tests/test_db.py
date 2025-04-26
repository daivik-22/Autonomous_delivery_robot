import os
import pytest
from autobot_pkg import db

@pytest.fixture(autouse=True)
def temp_db(tmp_path, monkeypatch):
    # Point DB_PATH to a temporary file
    monkeypatch.setattr(db, "DB_PATH", str(tmp_path / "test.db"))
    yield

def test_init_db_creates_tables():
    db.init_db()
    assert os.path.exists(db.DB_PATH), "Database file should be created"

def test_add_and_check_tag():
    db.init_db()
    assert not db.check_tag("ABC"), "Tag should not exist initially"
    db.add_tag("ABC", "owner")
    assert db.check_tag("ABC"), "Tag should be found after insertion"

def test_add_delivery_and_verify():
    db.init_db()
    did = db.add_delivery("CODE1")
    # Wrong code should not verify
    assert not db.verify_code(did, "WRONG"), "Invalid code should fail"
    # Correct code verifies once
    assert db.verify_code(did, "CODE1"), "Valid code should succeed"
    # Cannot verify same code twice
    assert not db.verify_code(did, "CODE1"), "Code should not verify twice"
