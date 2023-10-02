import cv2


def _set_face(robot, face="look_down", activated=True, path="./baxter-python3/faces/"):
    """
    sets the robot face to a specific one 
    activated is a default true variable that can be passed with a global variable in your code if you want to deactivate the face change all together
    """
    if activated:
        path=path+str(face)+".jpg"
        print(path)
        img = cv2.imread(path)
        if img is not None:
            image = cv2.resize(img, (1024,600))
            robot._set_display_data(image)
        else:
            print("Face file not found!")

def _set_look(robot, look_direction="down", activated=True, path="./baxter-python3/faces/"):
    """
    sets the robot looking direction to a specific one (down, up, left, right, frontal, left_up,...)
    activated is a default true variable that can be passed with a global variable in your code if you want to deactivate the face change all together
    """
    _set_face(robot=robot,face="look_"+look_direction,activated=activated,path=path)
