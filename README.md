# Ring:bit Car Robot for RoboRuckus using the Mbits and Arduino.
## Materials
1. [Ring:bit Car V2](https://shop.elecfreaks.com/products/elecfreaks-micro-bit-ring-bit-v2-car-kit-without-micro-bit-board)
2. Optional: [Tracking Module](https://shop.elecfreaks.com/products/elecfreaks-tracking-module-use-with-ring-bit-car-v2). Needed for automated movement calibration.
3. [Mbits](https://www.elecrow.com/mbits.html) (Currently broken on their website, but can be purchased from their [official AliExpress](https://www.aliexpress.com/item/1005003540049324.html), [RobotShop](https://www.robotshop.com/products/elecrow-mbits-esp32-dev-board-based-on-letscode-scratch-30-arduino), or an [AliExpress reseller](https://www.aliexpress.com/item/1005005524784099.html))

## Assembly
Assemble the robot according to the instructions provided in the robot kit with the exception of the following:
1. The wheel servos should be installed backwards so that the wheels sit in the center of the robot instead of forward.
2. After assembly add some weight to the rear of the robot to prevent it from tipping forward.
![photo of assembled robot](https://github.com/RoboRuckus/roboruckus-bot-ringbit-mbits-arduino/blob/main/media/AssembledRobot.jpg)

## Programming
1. To program the robot, first download [Visual Studio Code](https://code.visualstudio.com/).
2. Next, follow the instructions to [install PlatformIO](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode).
3. Clone or download this repository to a folder.
4. Open the project folder with PlatfomIO in Visual Studio Code.
5. Connect the Mbits to the computer via USB.
6. Upload the code using the [PlatformIO toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode-toolbar).
