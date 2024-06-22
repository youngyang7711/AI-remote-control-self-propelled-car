# AI remote control self-propelled car
## Product features

1.可以辨識眼前是否出現障礙物

2.可以辨識障礙物是甚麼物體

3.當辨識到眼前出現障礙物的時候可以自動避開 障礙物

4.可以遠端遙控玩具車來避開障礙物

5.可以自動按照地上規劃的路徑來行駛

## Development board: AMB82 MINI

![image](https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/4ea3c48d-8147-492a-b614-2a875dad710c)

[RTL8735B](https://www.amebaiot.com/en/amebapro2/): 32-bit Arm v8M, up to 500MHz, 768KB ROM, 512KB RAM, 16MB Flash (MCM embedded DDR2/DDR3L up to 128MB) 802.11 a/b/g/n WiFi 2.4GHz/5GHz, BLE 5.1, NN Engine 0.4 TOPS, Crypto Engine, Audo Codec, …

* [Ameba Arduino](https://www.amebaiot.com/en/ameba-arduino-summary/)

* [Amebapro2 AMB82-mini Arduino Example Guides](https://www.amebaiot.com/en/amebapro2-amb82-mini-arduino-peripherals-examples)

* [Amebapro2 AMB82-mini Arduino getting started](https://www.amebaiot.com/en/amebapro2-amb82-mini-arduino-getting-started/) <br>

## Product Image

![image](https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/415f1c1c-4d12-4a3f-a8d7-d8db5291195d)

## Reinforcement learning for autonomous driving using RL-DQN

![image](https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/3dd3f0e5-f3f8-4923-ae89-e5fb9d6dfc64)


## Code
```
/*

 Example guide:
 https://www.amebaiot.com/en/amebapro2-arduino-neuralnework-face-recognition-unlock/

 Face registration commands
 --------------------------
 Point the camera at a target face and enter the following commands into the serial monitor,
 Register face:                       "REG={Name}"  Ensure that there is only one face detected in frame
 Remove face:                         "DEL={Name}"  Remove a registered face
 Reset registered faces:              "RESET"       Forget all previously registered faces
 Backup registered faces to flash:    "BACKUP"      Save registered faces to flash
 Restore registered faces from flash: "RESTORE"     Load registered faces from flash

 This example takes snapshot of unrecognised personnel after the Face Registration mode is turned off.

 NN Model Selection
 -------------------
 Select Neural Network(NN) task and models using modelSelect(nntask, objdetmodel, facedetmodel, facerecogmodel).
 Replace with NA_MODEL if they are not necessary for your selected NN Task.

 NN task
 =======
 OBJECT_DETECTION/ FACE_DETECTION/ FACE_RECOGNITION

 Models
 =======
 YOLOv3 model         DEFAULT_YOLOV3TINY   / CUSTOMIZED_YOLOV3TINY
 YOLOv4 model         DEFAULT_YOLOV4TINY   / CUSTOMIZED_YOLOV4TINY
 YOLOv7 model         DEFAULT_YOLOV7TINY   / CUSTOMIZED_YOLOV7TINY
 SCRFD model          DEFAULT_SCRFD        / CUSTOMIZED_SCRFD
 MobileFaceNet model  DEFAULT_MOBILEFACENET/ CUSTOMIZED_MOBILEFACENET
 No model             NA_MODEL

*/

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNFaceDetectionRecognition.h"
#include "VideoStreamOverlay.h"
#include <AmebaServo.h>
#include "AmebaFatFS.h"

#define CHANNELVID  0    // Channel for RTSP streaming
#define CHANNELJPEG 1    // Channel for taking snapshots
#define CHANNELNN   3    // RGB format video for NN only available on channel 3

// Customised resolution for NN
#define NNWIDTH  576
#define NNHEIGHT 320

// Pin Definition
#define RED_LED                3
#define GREEN_LED              4
#define BACKUP_FACE_BUTTON_PIN 5
#define EN_REGMODE_BUTTON_PIN  6
#define SERVO_PIN              8

VideoSetting configVID(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configJPEG(VIDEO_FHD, CAM_FPS, VIDEO_JPEG, 1);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNFaceDetectionRecognition facerecog;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerFDFR(1, 1);
StreamIO videoStreamerRGBFD(1, 1);
AmebaServo myservo;

char ssid[] = "Chen的iPhone";    // your network SSID (name)
char pass[] = "BBCHEN710079";        // your network password
int status = WL_IDLE_STATUS;

bool doorOpen = false;
bool backupButtonState = false;
bool RegModeButtonState = false;
bool regMode = false;
uint32_t img_addr = 0;
uint32_t img_len = 0;
String fileName;
long counter = 0;

// File Initialization
AmebaFatFS fs;

void setup()
{
    // GPIO Initialisation
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BACKUP_FACE_BUTTON_PIN, INPUT);
    pinMode(EN_REGMODE_BUTTON_PIN, INPUT);
    myservo.attach(SERVO_PIN);

    Serial.begin(115200);

    // Attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }

    // Configure camera video channels with video format information
    Camera.configVideoChannel(CHANNELVID, configVID);
    Camera.configVideoChannel(CHANNELJPEG, configJPEG);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
    rtsp.configVideo(configVID);
    rtsp.begin();

    // Configure Face Recognition model
    facerecog.configVideo(configNN);
    facerecog.modelSelect(FACE_RECOGNITION, NA_MODEL, DEFAULT_SCRFD, DEFAULT_MOBILEFACENET);
    facerecog.begin();
    facerecog.setResultCallback(FRPostProcess);

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNELVID));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNELVID);
    Camera.channelBegin(CHANNELJPEG);

    // Configure StreamIO object to stream data from RGB video channel to face detection
    videoStreamerRGBFD.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerRGBFD.setStackSize();
    videoStreamerRGBFD.setTaskPriority();
    videoStreamerRGBFD.registerOutput(facerecog);
    if (videoStreamerRGBFD.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNELVID, configVID);
    OSD.begin();

    // Restore any registered faces saved in flash
    facerecog.restoreRegisteredFace();

    // Servo moves to position the angle 180 degree (LOCK CLOSE)
    myservo.write(180);
}

void loop()
{
    backupButtonState = digitalRead(BACKUP_FACE_BUTTON_PIN);
    RegModeButtonState = digitalRead(EN_REGMODE_BUTTON_PIN);

    if ((backupButtonState == HIGH) && (regMode == true)) {    // Button is pressed when registration mode is on
        for (int count = 0; count < 3; count++) {
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
            delay(500);
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            delay(500);
        }
        facerecog.backupRegisteredFace();    // back up registered faces to flash
        regMode = false;                     // Off registration mode
    }

    if (Serial.available() > 0) {
        String input = Serial.readString();
        input.trim();

        if (regMode == true) {
            if (input.startsWith(String("REG="))) {
                String name = input.substring(4);
                facerecog.registerFace(name);
            } else if (input.startsWith(String("DEL="))) {
                String name = input.substring(4);
                facerecog.removeFace(name);
            } else if (input.startsWith(String("RESET"))) {
                facerecog.resetRegisteredFace();
            } else if (input.startsWith(String("BACKUP"))) {
                facerecog.backupRegisteredFace();
            } else if (input.startsWith(String("RESTORE"))) {
                facerecog.restoreRegisteredFace();
            }
        }
    }

    if (regMode == false) {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        if ((RegModeButtonState == HIGH)) {
            regMode = true;
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
        }
    } else {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    }

    // Take snapshot and open door for 10 seconds
    if ((doorOpen == true) && (regMode == false)) {
        fs.begin();
        File file = fs.open(String(fs.getRootPath()) + fileName + String(++counter) + ".jpg");

        delay(1000);
        Camera.getImage(CHANNELJPEG, &img_addr, &img_len);
        file.write((uint8_t *)img_addr, img_len);
        file.close();
        fs.end();
        myservo.write(0);
        Serial.println("Opening Door!");

        delay(10000);
        myservo.write(180);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        doorOpen = false;
    }

    delay(2000);
    OSD.createBitmap(CHANNELVID);
    OSD.update(CHANNELVID);
}

// User callback function for post processing of face recognition results
void FRPostProcess(std::vector<FaceRecognitionResult> results)
{
    uint16_t im_h = configVID.height();
    uint16_t im_w = configVID.width();

    printf("Total number of faces detected = %d\r\n", facerecog.getResultCount());
    OSD.createBitmap(CHANNELVID);

    if (facerecog.getResultCount() > 0) {
        if (regMode == false) {
            if (facerecog.getResultCount() > 1) {    // Door remain close when more than one face detected
                doorOpen = false;
                digitalWrite(RED_LED, HIGH);
                digitalWrite(GREEN_LED, LOW);
            } else {
                FaceRecognitionResult face = results[0];
                if (String(face.name()) == String("unknown")) {    // Door remain close when unknown face detected
                    doorOpen = false;
                    digitalWrite(RED_LED, HIGH);
                    digitalWrite(GREEN_LED, LOW);
                } else {    // Door open when a single registered face detected
                    doorOpen = true;




                    digitalWrite(RED_LED, LOW);
                    digitalWrite(GREEN_LED, HIGH);
                    fileName = String(face.name());
                }
            }
        }
    }

    for (int i = 0; i < facerecog.getResultCount(); i++) {
        FaceRecognitionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);





        uint32_t osd_color;

        // Draw boundary box
        if (String(item.name()) == String("unknown")) {
            osd_color = OSD_COLOR_RED;
        } else {
            osd_color = OSD_COLOR_GREEN;
        }
        printf("Face %d name %s:\t%d %d %d %d\n\r", i, item.name(), xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNELVID, xmin, ymin, xmax, ymax, 3, osd_color);

        // Print identification text above boundary box
        char text_str[40];
        snprintf(text_str, sizeof(text_str), "Face:%s", item.name());
        OSD.drawText(CHANNELVI
```





## Demo 
### [遙控](https://youtu.be/w4kPHU47Nuc) <br>
### (https://www.youtube.com/watch?v=lF9Uk5XS0rA) <br>
### [玩具車自動避開障礙物](https://www.youtube.com/shorts/8DNJ__5eH_o) <br>


玩具車自動避開障礙物

https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/8f412978-ef1e-4212-b5ea-3354f7dc0e85

利用手機遙控

https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/f6ca249a-fbb8-44ae-8579-ad128bab4f61



https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/54152372-8381-4643-96eb-7d7c59bfe777

### AI face recognition
## Code
```
/*

 Example guide:
 https://www.amebaiot.com/en/amebapro2-arduino-neuralnework-face-recognition-unlock/

 Face registration commands
 --------------------------
 Point the camera at a target face and enter the following commands into the serial monitor,
 Register face:                       "REG={Name}"  Ensure that there is only one face detected in frame
 Remove face:                         "DEL={Name}"  Remove a registered face
 Reset registered faces:              "RESET"       Forget all previously registered faces
 Backup registered faces to flash:    "BACKUP"      Save registered faces to flash
 Restore registered faces from flash: "RESTORE"     Load registered faces from flash

 This example takes snapshot of unrecognised personnel after the Face Registration mode is turned off.

 NN Model Selection
 -------------------
 Select Neural Network(NN) task and models using modelSelect(nntask, objdetmodel, facedetmodel, facerecogmodel).
 Replace with NA_MODEL if they are not necessary for your selected NN Task.

 NN task
 =======
 OBJECT_DETECTION/ FACE_DETECTION/ FACE_RECOGNITION

 Models
 =======
 YOLOv3 model         DEFAULT_YOLOV3TINY   / CUSTOMIZED_YOLOV3TINY
 YOLOv4 model         DEFAULT_YOLOV4TINY   / CUSTOMIZED_YOLOV4TINY
 YOLOv7 model         DEFAULT_YOLOV7TINY   / CUSTOMIZED_YOLOV7TINY
 SCRFD model          DEFAULT_SCRFD        / CUSTOMIZED_SCRFD
 MobileFaceNet model  DEFAULT_MOBILEFACENET/ CUSTOMIZED_MOBILEFACENET
 No model             NA_MODEL

*/

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNFaceDetectionRecognition.h"
#include "VideoStreamOverlay.h"
#include <AmebaServo.h>
#include "AmebaFatFS.h"

#define CHANNELVID  0    // Channel for RTSP streaming
#define CHANNELJPEG 1    // Channel for taking snapshots
#define CHANNELNN   3    // RGB format video for NN only available on channel 3

// Customised resolution for NN
#define NNWIDTH  576
#define NNHEIGHT 320

// Pin Definition
#define RED_LED                3
#define GREEN_LED              4
#define BACKUP_FACE_BUTTON_PIN 5
#define EN_REGMODE_BUTTON_PIN  6
#define SERVO_PIN              8

VideoSetting configVID(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configJPEG(VIDEO_FHD, CAM_FPS, VIDEO_JPEG, 1);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNFaceDetectionRecognition facerecog;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerFDFR(1, 1);
StreamIO videoStreamerRGBFD(1, 1);
AmebaServo myservo;

char ssid[] = "Chen的iPhone";    // your network SSID (name)
char pass[] = "BBCHEN710079";        // your network password
int status = WL_IDLE_STATUS;

bool doorOpen = false;
bool backupButtonState = false;
bool RegModeButtonState = false;
bool regMode = false;
uint32_t img_addr = 0;
uint32_t img_len = 0;
String fileName;
long counter = 0;

// File Initialization
AmebaFatFS fs;

void setup()
{
    // GPIO Initialisation
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BACKUP_FACE_BUTTON_PIN, INPUT);
    pinMode(EN_REGMODE_BUTTON_PIN, INPUT);
    myservo.attach(SERVO_PIN);

    Serial.begin(115200);

    // Attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }

    // Configure camera video channels with video format information
    Camera.configVideoChannel(CHANNELVID, configVID);
    Camera.configVideoChannel(CHANNELJPEG, configJPEG);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
    rtsp.configVideo(configVID);
    rtsp.begin();

    // Configure Face Recognition model
    facerecog.configVideo(configNN);
    facerecog.modelSelect(FACE_RECOGNITION, NA_MODEL, DEFAULT_SCRFD, DEFAULT_MOBILEFACENET);
    facerecog.begin();
    facerecog.setResultCallback(FRPostProcess);

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNELVID));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNELVID);
    Camera.channelBegin(CHANNELJPEG);

    // Configure StreamIO object to stream data from RGB video channel to face detection
    videoStreamerRGBFD.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerRGBFD.setStackSize();
    videoStreamerRGBFD.setTaskPriority();
    videoStreamerRGBFD.registerOutput(facerecog);
    if (videoStreamerRGBFD.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNELVID, configVID);
    OSD.begin();

    // Restore any registered faces saved in flash
    facerecog.restoreRegisteredFace();

    // Servo moves to position the angle 180 degree (LOCK CLOSE)
    myservo.write(180);
}

void loop()
{
    backupButtonState = digitalRead(BACKUP_FACE_BUTTON_PIN);
    RegModeButtonState = digitalRead(EN_REGMODE_BUTTON_PIN);

    if ((backupButtonState == HIGH) && (regMode == true)) {    // Button is pressed when registration mode is on
        for (int count = 0; count < 3; count++) {
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
            delay(500);
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            delay(500);
        }
        facerecog.backupRegisteredFace();    // back up registered faces to flash
        regMode = false;                     // Off registration mode
    }

    if (Serial.available() > 0) {
        String input = Serial.readString();
        input.trim();

        if (regMode == true) {
            if (input.startsWith(String("REG="))) {
                String name = input.substring(4);
                facerecog.registerFace(name);
            } else if (input.startsWith(String("DEL="))) {
                String name = input.substring(4);
                facerecog.removeFace(name);
            } else if (input.startsWith(String("RESET"))) {
                facerecog.resetRegisteredFace();
            } else if (input.startsWith(String("BACKUP"))) {
                facerecog.backupRegisteredFace();
            } else if (input.startsWith(String("RESTORE"))) {
                facerecog.restoreRegisteredFace();
            }
        }
    }

    if (regMode == false) {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        if ((RegModeButtonState == HIGH)) {
            regMode = true;
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
        }
    } else {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    }

    // Take snapshot and open door for 10 seconds
    if ((doorOpen == true) && (regMode == false)) {
        fs.begin();
        File file = fs.open(String(fs.getRootPath()) + fileName + String(++counter) + ".jpg");

        delay(1000);
        Camera.getImage(CHANNELJPEG, &img_addr, &img_len);
        file.write((uint8_t *)img_addr, img_len);
        file.close();
        fs.end();
        myservo.write(0);
        Serial.println("Opening Door!");

        delay(10000);
        myservo.write(180);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        doorOpen = false;
    }

    delay(2000);
    OSD.createBitmap(CHANNELVID);
    OSD.update(CHANNELVID);
}

// User callback function for post processing of face recognition results
void FRPostProcess(std::vector<FaceRecognitionResult> results)
{
    uint16_t im_h = configVID.height();
    uint16_t im_w = configVID.width();

    printf("Total number of faces detected = %d\r\n", facerecog.getResultCount());
    OSD.createBitmap(CHANNELVID);

    if (facerecog.getResultCount() > 0) {
        if (regMode == false) {
            if (facerecog.getResultCount() > 1) {    // Door remain close when more than one face detected
                doorOpen = false;
                digitalWrite(RED_LED, HIGH);
                digitalWrite(GREEN_LED, LOW);
            } else {
                FaceRecognitionResult face = results[0];
                if (String(face.name()) == String("unknown")) {    // Door remain close when unknown face detected
                    doorOpen = false;
                    digitalWrite(RED_LED, HIGH);
                    digitalWrite(GREEN_LED, LOW);
                } else {    // Door open when a single registered face detected
                    doorOpen = true;
                    digitalWrite(RED_LED, LOW);
                    digitalWrite(GREEN_LED, HIGH);
                    fileName = String(face.name());
                }
            }
        }
    }

    for (int i = 0; i < facerecog.getResultCount(); i++) {
        FaceRecognitionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        uint32_t osd_color;

        // Draw boundary box
        if (String(item.name()) == String("unknown")) {
            osd_color = OSD_COLOR_RED;
        } else {
            osd_color = OSD_COLOR_GREEN;
        }
        printf("Face %d name %s:\t%d %d %d %d\n\r", i, item.name(), xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNELVID, xmin, ymin, xmax, ymax, 3, osd_color);

        // Print identification text above boundary box
        char text_str[40];
        snprintf(text_str, sizeof(text_str), "Face:%s", item.name());
        OSD.drawText(CHANNELVI
```
### demo

![S__13451274](https://github.com/toast517/AI-remote-control-self-propelled-car/assets/163410120/4a723d42-f544-4e14-8d7e-8cdd04475e1d)

### 組員: 
陳宗奕、楊易、林楷恩、陳宏宇、陳鋥錡、張仲翔
