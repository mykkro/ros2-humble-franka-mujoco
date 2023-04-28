# app.py

import yaml
from flask import Flask, render_template, jsonify, make_response


app = Flask(__name__, static_url_path='', static_folder='static')


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=5000)