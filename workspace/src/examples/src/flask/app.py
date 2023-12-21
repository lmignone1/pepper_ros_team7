from flask import Flask, render_template, send_from_directory
import os

PATH = os.path.join(os.path.dirname(__file__), 'payload.txt')

app = Flask(__name__)

@app.route('/dialogue',  methods=['GET'])
def dialogue():
    with open(PATH, 'r') as f:
        text = f.read()
    return render_template('dialogue.html', data = text)

@app.route('/static/index', methods=['GET'])
def index():
    return render_template('index.html', data = 'index.png') 


@app.route('/engagement',  methods=['GET'])
def engagement():
    return render_template('engagement.html')


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
