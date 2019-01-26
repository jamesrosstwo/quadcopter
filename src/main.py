import json
import os

from watson_developer_cloud import AssistantV2

api_key = os.environ.get("ASSISTANT_APIKEY")
api_url = os.environ.get("ASSISTANT_URL")
print(api_url)
assistant_id = "65ab6c6b-0296-4761-bca2-168415b00e7e"
if not api_key:
    print("Missing API key. Exiting.")
    exit()

assistant = AssistantV2(version="2018-09-20",
                        url=api_url,
                        username="apikey",
                        password=api_key)

session = assistant.create_session(assistant_id).get_result()
print(json.dumps(session, indent=2))


