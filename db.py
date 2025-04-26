# src/autopkg/db.py
import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).parent.parent / "autobot.db"

def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    # Authorized RFID tags table
    c.execute("""
      CREATE TABLE IF NOT EXISTS tags (
        uid TEXT PRIMARY KEY,
        owner TEXT
      )
    """)
    # Deliveries & confirmation codes
    c.execute("""
      CREATE TABLE IF NOT EXISTS deliveries (
        delivery_id INTEGER PRIMARY KEY AUTOINCREMENT,
        code TEXT UNIQUE,
        status TEXT CHECK(status IN ('pending','completed')) NOT NULL DEFAULT 'pending'
      )
    """)
    conn.commit()
    conn.close()

def add_tag(uid: str, owner: str):
    conn = sqlite3.connect(DB_PATH)
    conn.execute("INSERT OR IGNORE INTO tags(uid, owner) VALUES(?, ?)", (uid, owner))
    conn.commit()
    conn.close()

def check_tag(uid: str) -> bool:
    conn = sqlite3.connect(DB_PATH)
    cur = conn.execute("SELECT 1 FROM tags WHERE uid = ?", (uid,))
    exists = cur.fetchone() is not None
    conn.close()
    return exists

def add_delivery(code: str) -> int:
    conn = sqlite3.connect(DB_PATH)
    cur = conn.execute("INSERT INTO deliveries(code) VALUES(?)", (code,))
    conn.commit()
    delivery_id = cur.lastrowid
    conn.close()
    return delivery_id

def verify_code(delivery_id: int, code: str) -> bool:
    conn = sqlite3.connect(DB_PATH)
    cur = conn.execute("""
      SELECT 1 FROM deliveries WHERE delivery_id = ? AND code = ? AND status = 'pending'
    """, (delivery_id, code))
    ok = cur.fetchone() is not None
    if ok:
        conn.execute("UPDATE deliveries SET status = 'completed' WHERE delivery_id = ?", (delivery_id,))
        conn.commit()
    conn.close()
    return ok
