# The Helping Hand
## The "Hand" is here to help you in any situation, no matter how dangerous.

![alt text](docs/logo.jpg)

The HelpingHand can be used with a variety of applications, with our simple design the arm can be easily mounted on a wall or a rover. With the simple controls any user will find using the HelpingHand easy and intuitive. Our high speed video camera will allow the user to see the arm and its environment so users can remotely control our hand.

## Development
- Python openCV library
- 6 degree of freedom robotic arm
- Convoluted Neural Network using tensorflow
- Data set from Kaggle
- Arduino C
- PWM servo motor control

### Prereq
- opencv-python
- pyserial

## Directory
- `docs` website files
- `depth_camera` using the CNN
- `servo1` Arduino code


## Usage
To use the main system with the green ball controller, run `ball_tracking.py` (main controller script) and `cam.py` (which runs the camera attached to the arm)