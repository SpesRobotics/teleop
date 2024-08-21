from flask import Flask, send_from_directory
import ssl
from flask import request

app = Flask(__name__)


@app.route('/<path:filename>')
def serve_file(filename):
    return send_from_directory('.', filename)


@app.route('/pose')
def pose():
    x = request.args.get('x')
    y = request.args.get('y')
    z = request.args.get('z')
    print(x, y, z)


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

