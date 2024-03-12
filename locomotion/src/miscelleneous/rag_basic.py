import requests
import json
import ast


user_input = input("Enter command: ")

prompt = """
You are the joystick controller of a robot with an update rate of 10Hz. Please infer the output.
The output is a 5-dimensional vector, with the following values: [x-velocity, y-velocity, yaw-velocity, mode, gait]. 
x-velocity, y-velocity, and yaw-velocity have a range of [0,1] meters/second. 
mode can be a number from [0,1,2], where 0 corresponds to the motion that takes the robot from sleep state to stance state, 1 corresponds the motion that takes the robot from stance state to sleep state, and 2 corresponds to the condition when the robot is in stance and ready to accept velocity commands. The robot should change the mode to 2 if it is to move. The robot cannot move in modes 0 and 1. 
gait can be an integer from [0,1]. 0 represents stance gait and 1 represents trot gait.
The user will give desired motion as a textual input. They robot should stop and go into stance gait and mode if the user tell you to stop.
The output should be of the form: 
[x-velocity, y-velocity, yaw-velocity, mode, gait]
Replase the variables with appropriate numbers in the output. Please do not output other redundant words. Only output the command list. Do not output any other words or explanation.
The current user input is: {user_input}
"""


url = 'http://localhost:11434/api/generate'
data = {
    "model": "llama2",
    "prompt": prompt.format(user_input=user_input)
}
headers = {'Content-type': 'application/json'}

def main():
    global user_input
    while(user_input != "exit"):
        full_response = []
        user_input = input("Enter command: ")
        data["prompt"] = prompt.format(user_input=user_input)
        # print("Current prompt: ", data["prompt"])
        response = requests.post(url, data=json.dumps(data), headers=headers, stream=True)
        try:
            count = 0
            for line in response.iter_lines():
                if line:
                    decoded_line = json.loads(line.decode('utf-8'))
                    full_response.append(decoded_line['response'])
        finally:
            response.close()
        print("response: ", ''.join(full_response))
        print("Type: ", type(full_response[0]))
        float_response = ast.literal_eval(''.join(full_response))
    print("Exit commander...")

if __name__=="__main__":
    main()
