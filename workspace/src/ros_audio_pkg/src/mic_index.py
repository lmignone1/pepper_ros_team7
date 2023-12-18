import speech_recognition as sr

r = sr.Recognizer()
print(sr.Microphone().list_microphone_names())

for i, mic in enumerate(sr.Microphone.list_microphone_names()):
    if 'ReSpeaker 4 Mic Array' in mic:
        print(i)
