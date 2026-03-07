#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

void sendControllerData(ControllerPtr ctl) {
    // Format: LX,LY,RX,RY,BRAKE,THROTTLE,BUTTONS,DPAD\n
    // Axes:     -511 to 512
    // Brake/Throttle: 0 to 1023
    // Buttons:  bitmask (uint16)
    // Dpad:     bitmask (uint8)
    Serial1.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
        ctl->axisX(),
        ctl->axisY(),
        ctl->axisRX(),
        ctl->axisRY(),
        ctl->brake(),
        ctl->throttle(),
        ctl->buttons(),
        ctl->dpad()
    );

    //DEBUGGING - PRINTS ESP32 DATA TO SERIAL MONITOR
    //Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
    //    ctl->axisX(), ctl->axisY(),
    //    ctl->axisRX(), ctl->axisRY(),
    //    ctl->brake(), ctl->throttle(),
    //    ctl->buttons(), ctl->dpad()
    //);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                sendControllerData(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, 8, 7);  // RX=GPIO8(D5), TX=GPIO7(D4)

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Serial.println("Bluepad32 ready");
}

void loop() {
    if (BP32.update())
        processControllers();
    delay(50);  // ~20Hz update rate, safe for Uno buffer
}