from src.watson import Watson

assistant_id = "65ab6c6b-0296-4761-bca2-168415b00e7e"
assistant = Watson(assistant_id)
for i in range(20):
    print(assistant.message(input()))