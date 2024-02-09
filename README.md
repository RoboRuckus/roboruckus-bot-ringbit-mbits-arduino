# Ring:bit Car Robot for RoboRuckus using the Mbits and Arduino.
## Features
1. Inexpensive
2. Easy to assemble
3. Gyroscope based navigation
4. Web-based firmware updates
5. Movement calibration with optional line-following tracking module

## Materials
1. [Ring:bit Car V2](https://shop.elecfreaks.com/products/elecfreaks-micro-bit-ring-bit-v2-car-kit-without-micro-bit-board)
2. Optional: [Tracking Module](https://shop.elecfreaks.com/products/elecfreaks-tracking-module-use-with-ring-bit-car-v2). Needed for automated movement calibration.
3. [Mbits](https://www.elecrow.com/mbits.html) (Currently broken on their website, but can be purchased from their [official AliExpress](https://www.aliexpress.com/item/1005003540049324.html), [RobotShop](https://www.robotshop.com/products/elecrow-mbits-esp32-dev-board-based-on-letscode-scratch-30-arduino), or an [AliExpress reseller](https://www.aliexpress.com/item/1005005524784099.html). You can also [contact Elecrow directly](https://www.elecrow.com/contacts) to request an order of Mbits).


## Assembly
Assemble the robot according to the instructions provided in the robot kit with the exception of the following:
1. The wheel servos should be installed backwards so that the wheels sit in the center of the robot instead of forward.
2. After assembly add some weight to the rear of the robot to prevent it from tipping forward.
![photo of assembled robot](/media/AssembledRobot.jpg)

## Programming
1. To program the robot, first download [Visual Studio Code](https://code.visualstudio.com/).
2. Next, follow the instructions to [install PlatformIO](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode).
3. Clone or download this repository to a folder.
4. Open the project folder with PlatformIO in Visual Studio Code.
5. Connect the Mbits to the computer via USB.
6. Upload the code using the [PlatformIO toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode-toolbar).

## Operation
Using the robot is, for the most part, very simple, just turn it on! However, there is some first-time setup you'll need to do, as well as some advanced tuning options, which are all detailed below.

### The A and B buttons
The A and B buttons on the Mbits have several functions depending on when and how they are pressed:

#### Hold A During Boot
This will reset the currently saved Wi-Fi and game server settings. When this is accomplished, the robot will display a check-mark and reboot, you should release the button when you see the check-mark. See [Connecting to the Game](#connecting-to-the-game).

#### Hold B During Boot
This will have the robot enter the movement calibration mode. When this is accomplished, the robot will display a duck symbol, you should release the button when you see the duck symbol. See [Robot Movement Calibration](#robot-movement-calibration).

#### Press A
Pressing the A button any time after the robot has successfully connected to the game server will have it recalibrate the onboard gyroscope. This is automatically done when the robot is powered on, but if the robot is drifting or not turning properly, this can be repeated to help. When the A button is pressed the robot will display a duck symbol during calibration, then display the previous image when the calibration is finished. **The robot should be kept perfectly still during the calibration process.** This can be done anytime, even while playing the game, but shouldn't be done during the movement phase of the game.

#### Press B
Pressing the B button any time after the robot has successfully connected to the game server will have it display the last octet of its IP address on the screen, one number at a time. This can be useful for troubleshooting or for connecting to the robot to [update the firmware](#updating-the-firmware).

### Connecting to the Game
If this is the first time powering on the robot it may take a while as it needs to format and mount the SPIFFS. Once it's finished the initial boot, you'll need to configure it to connect to the Wi-Fi network used by the game server as well as the game server's IP address. When you power on the robot for the first time (or if the expected Wi-Fi network is not available) you'll need to wait a minute or so until the screen displays a duck symbol. This is the symbol used to indicate that the robot is in a setup mode.

Once in the Wi-Fi setup mode, the robot will create its own Wi-Fi network named "Ruckus_XXXXXXX" where the X's will be a string of numbers and letters. Connect to that network (default password is RuckusBot), ideally with your phone, and visit the IP address `192.168.4.1` to reach the robot's setup interface (a phone may do this automatically). Choose "Configure Wi-Fi" to setup the robot, and pick the Wi-Fi network you want the robot to connect to. Enter the Wi-Fi password, IP address of the game server, and the port used by the game server, as shown below.

![Wi-Fi Gateway Screenshot](/media/RobotWifiGateway.png)

If the Wi-Fi credentials are good, the robot will reboot and attempt to connect to the game server, and you're done! To reset the Wi-Fi or game server settings, see [Hold A During Boot](#hold-a-during-boot).

### Playing With the Robot
Turn the robot on, keep it perfectly still during the boot process to properly calibrate the gyroscope. If the game server and Wi-Fi network are working, the robot will automatically connect and display a happy face. If the robot can't connect to the game server, it will show a sad face and keep trying. Once all the robots you need are connected, you can refer to [this documentation](https://www.roboruckus.com/documentation/running-a-game/) for how to tune their movement and setup the game.

### Robot Tuning Parameters
The following tuning parameters are available for this robot (see [tuning a robot](https://www.roboruckus.com/documentation/running-a-game/#Tuning_the_Robots)):
* Drift Limit: This is the number of degrees the gyroscope will allow a robot to drift off a linear course before correcting.
* Drift Boost: This is how aggressively the robot will move to correct itself when it drifts off course.
* Left Backward Speed: This is the speed of the left wheel when moving backwards. The smaller this number, the faster the movement.
* Left Forward Speed: This is the speed of the left wheel when moving forwards. The larger this number, the faster the movement.
* Left Backward Speed: This is the speed of the right wheel when moving backwards. The larger this number, the faster the movement.
* Left Forward Speed: This is the speed of the left wheel when moving forwards. The smaller this number, the faster the movement.
* Linear Move Time: This is the time, in milliseconds, that the robot needs to move a distance of one board square.
* Turn Angle: This is the actual number of degrees the gyroscope needs to measure to have the robot complete a 90-degree turn.
* Robot Color: The color displayed on the robot's LEDs.
* Robot Name: The robot's name.

### Robot Movement Calibration
Print out the [calibration pattern](/CalibrationPattern.pdf) on a US letter sized piece of paper (8.5"x11"), you can optionally 3D the [robot tuning jig](/Robot_Tuning_Jig.stl) to help with this process.
1. Turn off the robot and adjust the switch on the bottom to "Other Module".
2. While holding the B button turn on the robot.
3. Orient the calibration paper so the black bar near the edge is at the "top".
4. When the screen shows a duck, place the robot with the wheels on the edge of the "bottom" of the paper.
5. Press the A button and wait. The robot will drive forward with three possible results:
   1. The robot isn't driving straight and veers off course. If this happens the robot will stop and make adjustments. Return to step 5.
   2. The robot will drive the length of the paper but the screen doesn't show a check mark, the robot will make a speed adjustment. Return to step 5.
   3. The robot will drive the length of the paper and show a check mark. Linear movement is calibrated, proceed to step 6, or skip to step 8.
6. Place the robot on the center of the paper so that the wheels sit on the lines on either side of the intersection.
7. Press the B button and wait for the robot to complete a full rotation. If the robot's movement seems unsatisfactory, you may repeat this process from step 6 until you're happy.
9. Power off the the robot, and toggle the bottom switch back to "Rainbow LED".

### Updating the Firmware
You can update the robot's firmware any time after it has connected to the Wi-Fi network (usually after it displays a happy or sad face). Simply connect to the same Wi-Fi network as the robot and enter the robot's IP address in your browser. Once connected, select the appropriate `firmware.bin` file and start the update. Be patient as the robot updates and reboots. All the robot's settings should be preserved.
