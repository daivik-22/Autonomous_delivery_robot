import pytest
from autobot_pkg.delivery_api import app
from autobot_pkg import db

@pytest.fixture(autouse=True)
def temp_db(tmp_path, monkeypatch):
    # Use a clean DB for each test
    monkeypatch.setattr(db, "DB_PATH", str(tmp_path / "test_api.db"))
    db.init_db()
    yield

@pytest.fixture
def client():
    return app.test_client()

def test_issue_endpoint(client):
    resp = client.post("/issue")
    assert resp.status_code == 201
    data = resp.get_json()
    assert "delivery_id" in data and "code" in data

def test_confirm_endpoint(client):
    # Issue first to get valid code
    issue_resp = client.post("/issue")
    did, code = issue_resp.get_json().values()
    # Wrong code returns 400
    bad = client.post("/confirm", json={"delivery_id": did, "code": "BAD"})
    assert bad.status_code == 400
    # Correct code returns 200
    good = client.post("/confirm", json={"delivery_id": did, "code": code})
    assert good.status_code == 200
    assert good.get_json().get("status") == "unlocked"
