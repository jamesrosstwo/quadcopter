from bottle import route, run, template


def start():
    run(host='0.0.0.0', port=8080)


@route('/')
def index():
    with open('readings.json') as f:
        readings = f.read()
    return template(readings)


if __name__ == "__main__":
    run(host='0.0.0.0', port=8080, debug=True)
