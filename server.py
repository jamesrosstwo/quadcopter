from bottle import route, run, template


class Server:

    def __init__(self, data):
        self.template = data

    @route('/')
    def index(self):
        return template(self.template)


    def start(self):
        run(host='0.0.0.0', port=8080)


if __name__ == "__main__":
    run(host='0.0.0.0', port=8080)
