# src/autopkg/delivery_api.py
from flask import Flask, request, jsonify
from autopkg.db import init_db, add_delivery, verify_code

app = Flask(__name__)
init_db()

@app.route('/issue', methods=['POST'])
def issue():
    """Client posts nothing; server returns a new delivery_id + random code."""
    import secrets
    code = secrets.token_hex(3)  # e.g. '9f4b3c'
    delivery_id = add_delivery(code)
    return jsonify({'delivery_id': delivery_id, 'code': code}), 201

@app.route('/confirm', methods=['POST'])
def confirm():
    """
    Expect JSON: { "delivery_id": <int>, "code": "<string>" }
    Returns 200 if valid; 400 if not.
    """
    data = request.get_json(force=True)
    did = data.get('delivery_id')
    code = data.get('code')
    if not (isinstance(did, int) and isinstance(code, str)):
        return jsonify({'error': 'bad request'}), 400
    if verify_code(did, code):
        return jsonify({'status': 'unlocked'}), 200
    return jsonify({'error': 'invalid code'}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
