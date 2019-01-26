import os

from watson_developer_cloud import AssistantV2


def connect():
    api_key = os.environ.get("ASSISTANT_APIKEY")
    api_url = os.environ.get("ASSISTANT_URL")
    if not api_key:
        raise NameError("Missing API key")

    return AssistantV2(version="2018-09-20",
                       url=api_url,
                       username="apikey",
                       password=api_key)


def get_session(assistant, assistant_id):
    session = assistant.create_session(assistant_id).get_result()
    return session


class Watson:
    def __init__(self, assistant_id):
        self.assistant_id = assistant_id
        self.assistant = connect()
        self.session = get_session(self.assistant, self.assistant_id)
        self.session_id = self.session["session_id"]

    def message(self, message):
        return self.assistant.message(
            self.assistant_id,
            self.session_id,
            input={'text': message},
            context={
                'metadata': {
                    'deployment': 'myDeployment'
                }
            }).get_result()["output"]["generic"][0]["text"]
