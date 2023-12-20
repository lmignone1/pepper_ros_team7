from flask import Flask, render_template
import os

app = Flask(__name__)

@app.route('/index1')
def index():
    path = os.path.join(os.path.dirname(__file__), 'data.txt')
    with open(path, 'r') as f:
        data = f.read()
        data = data.split()
    return render_template('index1.html', data = data)

@app.route('/index3')
def index3():
    return render_template('index3.html')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
