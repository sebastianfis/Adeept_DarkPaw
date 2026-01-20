# DarkPaw

My personal take on improving the Adeept Dark Paw robot. Specific features I aim to tackle:
* **Correct Kinematic model**
* **AI supported object recognition/tracking**
* **Create an autonomous mode with dedicated behaviour**
* **Update the robot control interface**
* ~~**Voice recognition**~~ -> abandoned for now
* ~~**Sound generation**~~ -> abandoned for now


## UPDATE 2026-01-20:
As usual, it has been quite some time since I wrote the last update :sweat_smile: : 

### Hardware and hardware integration:
Long term testing showed, that the original 25 W power supply I intended to use, is too weak to run the Raspi5 with AI 
Hat and the ESP and the LEDs and all the servos. So I switched the power supply for something bigger and added a third 
18060 battery to power the whole thing for a decent amount of time (integration of the batteries pending). 

This led to a new issue: The ESP would not boot at the same time as the Raspi, which was never a problem before 
(Either its voltage ripple or some weird grounding issue of the power supply). I was able to solve this by starting 
up the ESP and using one of its outputs to switch on the RASPI via a MOSFET relays, so they now boot in sequence.

I also added a watchdog to the communication, so the ESP would detect a software crash on the Raspi side and stop 
instead of coniuting the last given command!

### Correct Kinematic model
I did some quite extensive testing on the C++ Implementation. The ESP32 works like a charm and is very reliable!

### AI supported object recognition/tracking
I added some code for object movement detection and in the process discovered, how unreliable the distance sensor 
readings are. Since on the Raspi 5 the Chip architecture is fairly different, hardware timed GPIO is not availlable.
Since regular GPIO timing in Python is not the most precise, I had to switch the distance sensor implementation to 
gpiod and add a bunch of stuff (floating average mean, rejection of false trigger events, sanity checks on timing and 
disatnce values). This is now as good, as it's going to get while running directly on a Pi. I also discovered that 
regular updates to the selected target data were missing, which is mandatory for the movement check.

### Robot control Interface website
Tested now and fully functional.

### What's next?
I am quite satisfied with the hardware as it is now (battery integration still pending). Also the C++ implentation is 
rather solid now and the Robt Control interface website (while it is not the most beautiful site) is very functional 
and highly responsive. From the original project goals, only **Create an autonomous mode with dedicated behaviour** is 
not complete yet, so that is where i can finally put my focus on.

## UPDATE 2025-04-26:
It has been quite some time since I wrote the last update and quite a few things have changed in the meantime :sweat_smile: : 

### Hardware and hardware integration: 
I designed some 3d printed parts to implement the changes I had to do in terms of hardware positioning. Check out the 
3d planning folder for the models, if you think you can use them.

### Correct Kinematic model
I did not have the time for a proper test of the C++ Implementation, yet. All I know right now is, that the serial 
communication works, so the C++ programm on the ESP is running fine, but I have not seen any actual movement from 
the robot yet. Have to do more tests on an isolated part of the setup to bugfix this.

### AI supported object recognition/tracking
Using the RASPI 5 and the AI HAT is very cool. I can now run the newest yolov11m model on the Hailo with ~18 fps 
framerate! 

### Robot control Interface website
While running the detection code locally gave me pretty good framerates, they plumeted to ~ 3 fps, when I was using 
the Webserver. Clearly streaming a MJPEG to a flask server is not the best choice for this task. In the end, I was 
forced to go a completely different route. There is a good reason, that both Hailo and Raspi Camera have a good load 
of GStreamer examples in ther repos. Gstreamer is probably one of the best choices, if you want to stream actual video 
(in contrast to an image sequence) over the web. But the python bindings for that are not well documented and the 
string based pipeline concept can be hard to grasp in the beginning. Since Hailo has a repo, where they only use 
gstreamer instead of Python for the whole chain of captureing an image, runing inference, annotation, and finally 
displaying/streaming, I gave that implementation a try. It turned out to be slower and not as pretty, as the 
implementation I had before using Raspicam2 and Supervision code, so I quickly gave up on that. So the new architecture 
is the following:

Raspicam2 -> inference & frame annoation as videosrc -> Gstreamer Pipeline -> WebRTC as videosink -> display on webpage

Took me a while to get this running, but it finally does! Bonus of this: Through WebRTC, asyncio and aiohttp I can also 
create a VERY responsive data channel for sending measurement data to the control interface and receive comands from 
it's elements.

## UPDATE 2024-12-08:
It has been quite some time since I wrote the last update and quite a few things have changed in the meantime :sweat_smile: : 

### Hardware
Time flies and when I started this project, the Google Coral Dev Board Mini or the Google Coral USB Accelerator were 
the only viable options for hardware accelerated AI applications on a size compatible with the Dark Paw robot frame. 
Well, there was the NVidia Jetson Nano, but I found that one too expensive and a bit too power hungry.

With the Raspberry Pi 5 and the Hailo AI HAT now on the market, there are is now actually a better option availlable 
considering the following criteria:
* Computing performance (in terms of TOPS)
* Model input tensor size (= image resolution) for yolo models
* Compatibility with peripheral equipment 
* Development comfort for video applications on a non-headless system
* Product support: There seems to have been no update to the Mendel Linux OS for the past 3 years...

Since I was never really quite satisfied with the object detection performance I was able to achieve with the Coral Dev Board Mini, I've chosen to switch the hardware for a Raspi 5 with an AI HAT. I felt the 13 TOPS variant would be suffcient and would safe some power vs. the 26 TOPS variant. This does not change anything on the planned "fore brain" / "hind brain" separation. I still think it is a good idea to do that separation!

I also added a HC-SR04 ultrasonic distance sensor. This will allow the robot to move in on an object down to a certain distance and even keep the distance for a moving object. And to make the robot a bit more expressive, I exchanged the blue LED of the front lens with a WS2812b multi-colour LED, that can be chained to the internal LEDs of the robot.

### Hardware Integration
Switching to the Raspi 5 / AI HAT Architecture has one downside though: It will be a lot bigger, than the Coral Dev Board Mini. So I had to do some re-arrangements of the robots insides: The LED-PCBs have to be moved further outside. The RASPI 5 and the AI HAT will be positionend on one side of the vertical electronics holder, while the ESP32 and the I2C PWM Servo Controller will be positioned on the other side. It's a close fit and I will probably curse a lot during cabling of this, but it should fit. I also won't ba able to fit the power supply inside the robot anymore, so I added an add-on part for the robot to carry a new DF-robot power supply PCB on its aft-side (this should come in handy in case of a robot spider uprising :joy:). Finally I added a holder to the HC-SR04 ultrasonic distance sensor to the upper front of the robot.

### Correct Kinematic model
I was able to port the python implementation to C++ and run it on an ESP32. It runs without crashing, and first tests on the calculated values are very promising. However, it is still lacking a real life test with the servos connected.

For communcation I will try using the good old serial connection first. This will run with minimal overhead both on the Raspi and the ESP32.

### AI supported object recognition/tracking
I was never really quite satisfied with the performance of the object detection models of the Google Coral Model Zoo on the Google Coral Mini board. The models were either too slow for real time video detection or accuracy was not satisfactory. So I went looking for more cutting edge object detection models and stumbled upon the yolo model series and later also ssd_spaghettinet. However, with these newer models some internal architecture of the Edge TPU limits the input tensor size, with which the model will still compile to run entirely on the Edge TPU to rather low resolutions. This means that object detection performance was better than with the models from the Coral model zoo, but false detection rates were still higher than what I had hoped for. Also, the limited resolution would make the robot a bit "short-sighted". It did work with an acceptable frame rate and I even got a tracker akgorithm to work, whcih would allow for keeping IDs of objects across different frames.

In the meantime, HAILO released a dedicated AI HAT for the Raspberry Pi 5, which does not have this limitation on input tensor size for modern object detection architectures. It also exceeds the TPU computing power by a factor > 3 (13 TOPS on the HAILO-8L vs. 4 TOPS on the Google Coral Dev Board Mini). So I decided to switch hardware (see above) and go ahead with a Raspi 5 equipped with an AI HAT. Getting the models to run in the Raspi Eco system was rather straightforward based on the examples in the picamera2 Repo and the HAILO Raspi 5 examples. This inculdes a object tracking algorithm with really awesome performance. 

### Autonomous mode with dedicated behaviour
No update on this (yet), except that I included 
* A "patrol" mode: The robot will go back and forth, with some tolerance on the angles, so it won't always go back and forth on the same path
* A "dance" mode: LEDs will set a light show and the robot will assume different poses at a certain rythm. This one is actually already finished :joy:

### Robot control Interface website
Oh boy! I put that off for quite some time. Understanding how the code of the Adeept Dark Paw original interface works was almost impossible to me. At some point I understood, that it was built in some sort of .js environment (I think Vue.js), but that did not make it much easier for someone with 0 experience in frontend development. In the end I gave up on trying to understand what the original code was trying to do and decided to do my own thing. 

So I started with designing a html/css website with Grapes.js and wrote my own javascript middleware. Putting that in contact with a flask server and responsive video streaming was everything but easy. There are numerous examples for simple video streaming out there, but most will reload the entire webiste over and over again, making registration of input commands a gamble at best. I finally found a solution for that and wrote a class-wrapped webserver implementation with thread-save command and data passing.

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
I actually started working on that part by drawing out the logic in a block flow diagram, but it is far 
from being finished...

## UPDATE 2024-02-23:
### Correct Kinematic model
Ok, so Kinematics works like a charm and I managed to implement all possible poses. However, I noticed 
a few unexpecetd challanges:

* From the "standard" leg position, movement range in y is VERY limited. I don't think that there's 
much can be done about it. It's a result of the kinematics. The only workaround for that is to let it do smaller 
steps in faster succession in y direction (move left/right). This also means, that I will not be able to make 
the robot circle an external reference point (like an identified object, it finds interesting and wants to 
take a closer look).
* The COG of the robot seems to be badly balanced. The poses, where it lifts a single leg are very 
unstable (and this means it does wobble a lot when walking as well). I'll have to do some more tests, 
this is true for all legs and if it can be counterbalanced. Another explanation might be the inertia of 
setting a pose too quickly. This can be easily changed, as the implementation allows for intermediate 
samples.
