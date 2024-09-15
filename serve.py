from flask import Flask, send_from_directory
import ssl
from flask import request
from teleop_publisher import TeleopPublisher, ROSTeleopPublisher


app = Flask(__name__)
teleop = ROSTeleopPublisher()


@app.route('/<path:filename>')
def serve_file(filename):
    return send_from_directory('.', filename)


@app.route('/pose', methods=['POST'])
def pose():
    json_data = request.get_json()
    teleop.update(json_data)

    return {'status': 'ok'}


@app.route('/log', methods=['POST'])
def log():
    json_data = request.get_json()
    print(json_data)
    return {'status': 'ok'}


@app.route('/')
def index():
    return send_from_directory('.', 'index.html')


def run():
    context = ssl.SSLContext(ssl.PROTOCOL_TLS)
    context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")

    # Run the Flask app with SSL
    app.run(host='0.0.0.0', port=4443, ssl_context=context)


if __name__ == '__main__':
    run()

