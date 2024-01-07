from flask import Flask, render_template, request
import os


app = Flask(__name__)

@app.route('/dialogue',  methods=['GET']) # url for the dialogue page when the user speaks
def dialogue():
    text = request.args.get('text') # text passed trough url
    return render_template('dialogue.html', data = text)

@app.route('/static/index', methods=['GET'])    # url for the index page 
def index():
    return render_template('index.html', data = 'index.png') 


@app.route('/engagement',  methods=['GET']) # url for the engagement page when pepper engages the user
def engagement():
    return render_template('engagement.html')


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
