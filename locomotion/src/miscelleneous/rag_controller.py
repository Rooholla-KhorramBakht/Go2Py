import requests
import json
import rclpy
from sensor_msgs.msg import Joy
import ast

user_input = input("Enter command: ")

prompt = """
You are the joystick controller of a quadruped robot with an update rate of 10Hz. Please infer the output.
The output is a 5-dimensional vector, with the following values: [x-velocity, y-velocity, yaw-velocity, mode, gait]. 
x-velocity is the velocity in x-direction and has a range of [-1,1] meters/second, 
y-velocity is the velocity in y-direction and has a range of [-1,1] meters/second, 
yaw-velocity is the angular velocity about z-axis and has a range of [-1,1] radians/second, 
mode can be a number from [0,1,2], where 0 corresponds to the condition that takes the robot from sleep to stance, 1 corresponds the condition that takes the robot from stance to state, and 2 corresponds to the condition when the robot is in stance and ready to accept velocity commands. The robot should change the mode to 2 if it is to move. The robot cannot move in modes 0 and 1. mode values 0 and 1 are critical and mode should be changed to 0 or 1 only if the user has specified. Mode should remain 2 otherwise.
gait can be an integer from [0,1]. 0 represents stance gait and 1 represents trot gait. gait should be 1 if the velocities are non-zero. stance means stance gait and trot means trot gait
The robot always starts from sleep and cannot move unless it is in stance. mode can only be changed after user specifies. 
The user will give desired motion as a text input. The robot should set the velocities to zero and go into stance gait if the user tell you to stop.
The output should be of the following form only: 
[x-velocity, y-velocity, yaw-velocity, mode, gait]
Replace the variables with appropriate numbers in the output. Please do not output other redundant words. Do not explain anything. Only output the specified list.
The current user input is: {user_input}
"""


url = 'http://localhost:11434/api/generate'
data = {
    "model": "mixtral",
    "prompt": prompt.format(user_input=user_input)
}
headers = {'Content-type': 'application/json'}

def get_llm_command(usr_input):
    full_response = []
    data["prompt"] = prompt.format(user_input=usr_input)
    response = requests.post(url, data=json.dumps(data), headers=headers, stream=True)
    try:
        count = 0
        for line in response.iter_lines():
            if line:
                decoded_line = json.loads(line.decode('utf-8'))
                full_response.append(decoded_line['response'])
    finally:
        response.close()
    float_response = ast.literal_eval(''.join(full_response).strip(' '))
    # print(float_response)
    return float_response

def main():
    rclpy.init()
    
    name = "go2"
    
    joystick_data = Joy()
    joystick_data.axes = [0.] * 5
    command = []

    node = rclpy.create_node("llm_commander")
    jcmd_publisher = node.create_publisher(Joy, "/" + name + "/joystick_cmd", 1)

    global user_input
    try:
        while rclpy.ok():
            user_input = input("Enter command: ")
            if (user_input == "exit"):
                break
            command = get_llm_command(user_input)
            joystick_data.axes = [float(x) for x in command]
            jcmd_publisher.publish(joystick_data)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

if __name__=="__main__":
    main()
