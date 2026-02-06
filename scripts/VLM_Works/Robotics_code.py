import base64
import requests
import cv2
Order=False

cam = cv2.VideoCapture(1)
#infinite loop to run the program
while(True):
    #takes the instruction for the robot
    instruction = input("Enter your instruction: ")
    print(instruction)
    Order=True
    
    Ping = (requests.post("http://localhost:8080")).json()
    ServerPing = Ping["Ping"]
    #Loops only if there is an order for the robot
    while(Order==True):
        
        if(ServerPing=="True"):
            #captures a frame
            ret, frame = cam.read()
            if ret:
                cv2.imshow("Captured", frame)         
                cv2.imwrite("captured_image.png", frame)      
            else:
                print("Failed to capture image.")
            with open("captured_image.png", "rb") as f:
                img_b64 = base64.b64encode(f.read()).decode()
            #the prompt foundation the VLM 
            prompt = """
                You are controlling a Baxter robot. 
                Look at the image and output ONLY with these commands:

                LEFT | RIGHT | UP | DOWN | GRASP | RELEASE | STOP,

                Do not include any explanation.
            """
            #Uses a locally hosted model from the Ollama app to querey the model 
            response = requests.post(
                "http://localhost:11434/api/generate",
                json={
                    "model": "moondeam",
                    "prompt": prompt  + instruction,
                    "images": [img_b64],
                    "stream": False
                }
            )
            #sends the action to the robot
            action = response.json()["response"].strip()
            print("Action:", action)
            payload = {"Action": action}

            requests.post("http://localhost:8080", json=payload)
           
    Order=False