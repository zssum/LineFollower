from flask import Flask
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
app = Flask(__name__)

@app.route("/")
def hello():
    return app.send_static_file('controller.html')

@app.route("/f")
def f():
    ser.write("f")
    return "Forward"

@app.route("/b")
def b():
    ser.write("b")
    return "Backward"

@app.route("/l")
def l():
    ser.write("l")
    return "Left"

@app.route("/r")
def r():
    ser.write("r")
    return "Right"

@app.route("/s")
def s():
    ser.write("s")
    return "Stop"

if __name__ == "__main__":
    app.run(host="0.0.0.0")
