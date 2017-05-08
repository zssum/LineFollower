from flask import Flask
import serial
ser = serial.Serial('/dev/ttyACM1', 9600)
app = Flask(__name__)

@app.route("/")
def hello():
    ser.write("read")
    return "Hello World!"

@app.route("/g")
def g():
    ser.write("read1")
    return "Hello World!"

if __name__ == "__main__":
    app.run(host="0.0.0.0")
