# DarkPaw

My personal take on improving the Adeept Dark Paw robot. Specific features I aim to tackle:
* **Correct Kinematic model**
* **AI supported object recognition/tracking**
* **Voice recognition**
* **Sound generation**
* **Create an autonomous mode with dedicated behaviour**

## UPDATE 2024-02-11:
After abondening the project for quite some time I took it up again. Here are some thoughts I had in the meantime 
(some of those caused my abandoning the project originally :sweat_smile:): 
### Hardware
Its pretty clear that the original Raspi based hardware would not be able to sustain all the features I'd like 
the robot to have. Checking the capabilities of availlable hardware, I decided that I would go with a Google 
Coral Mini board instead as main computation device, due to its on-board TPU, making it a great choice for 
AI applications.

However, after playing around with the Google Coral board I realized, that even with the on-board TPU there
would be issues with realization of all the stuff I wanted to implement, mainly because it would not be able 
to run several pre-compiled models (object recognition/tracking + Voice recognition + Sound generation) in 
parallel, but also since even running object recognition/tracking alone is already rather slow on the board.

So I had to narrow the project down on the points that were most important to me: 
* **Correct Kinematic model**
* **AI supported object recognition/tracking**
* ~~**Voice recognition**~~ -> abandoned for now
* ~~**Sound generation**~~ -> abandoned for now
* **Create an autonomous mode with dedicated behaviour**

Also, python is not the ideal programming language for "number crunching" which is essential for a 
correct kinematic model. With the lags I had seen on the Coral board, I decided to give the robot a "hind brain". 
Think about it: When you want to lift your extended arm, you do not think: 
* check shoulder angle, 
* check elbow angle
* calculate angle difference shoulder
* calculate angle difference elbow
* execute necessary angle changes

All you think is: "Lift and extend arm". All the rest: the fine adjustments, the translations into explicit 
control signals to all the muscles involved, the correct timing of the movements, etc... that's all done by 
your hind brain, without you having to take care of it. Why not mimick that separation of tasks in the robot 
as well? 

So the plan is to share the load between two computing entities: The main behaviour algorithm and AI supported 
object recognition/tracking would run on the "fore brain", which is the Google Coral board. As "hind brain" I am 
thinking about using somthing like an ESP32 (tbc). Running the number crunching kinematics on the ESP in C++ 
makes it efficient and fast. From there, Servo control should be pretty straight froward through a I2C connected 
PWM controller.

Considering how the two "brain-parts" will communicate with each other, I thought about using a serial exchange. 
But maybe I'll use the ROS2 environment instead, since it already provides a framework for communication of 
different computing nodes.

### Correct Kinematic model
If you checked out the implementation size of the kinematic model, you might wonder why It is so huge. 

The answer is simple: The availlable robot control packages I found out there while researching (e.g. ROS2) are 
limited to linear kinematic models. This is well and good for most applications, but the Adeept Dark Paw robots 
construction relies heavily on four bar linkage mechanisms (check out [this page](https://www.geogebra.org/m/BueCG9ch)
for an interactive animation).

Such a mechanism can be extremely strong, but is **highly non-linear**, especially with several of these 
actuators linked together, as it is the case here. Also, there are limits to the acutators, where further 
increase of the actuated parameter (aka servo angle) would actually lead to a decrease of the desired effect 
(e.g. retraction of the foot, even though a linear model would predict further extension).

So that's why I had to write the code from scratch to get a truely correct kinematic model. 

The model was already working quite well in Python, when I stopped working on the project, but I 
decided to add some details to the test visualizations (e.g. making them full 3d and build up on one another).

The next step will be to re-implement the tested python code in C++. In principle easy, but I have not yet 
written code containing class definitions in C++. But I'm hoping ChatGPT will help me out there :grin:

### AI supported object recognition/tracking
I already played around a bit with the Google Coral board and think it should work. I just have to find out, 
how I want to do the integration with the overall behavior algorithms, but more on that later

### Autonomous mode with dedicated behaviour
I actually started working on that part by drawing out the logic in a block flow diagram, but it is far from being 
finished...

