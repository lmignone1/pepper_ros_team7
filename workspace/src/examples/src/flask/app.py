from flask import Flask, render_template

app = Flask(__name__)

@app.route('/index1')
def index():
    return render_template('index1.html')

@app.route('/index3')
def index3():
    return render_template('index3.html')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
