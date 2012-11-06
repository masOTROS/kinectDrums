#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBlobTracker.h"

#define BACKGROUND_FRAMES 100

#define SOUNDS 5
#define ZONES SOUNDS

class testApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void touchOn( ofxBlob& b );
    void touchMoved( ofxBlob& b );
    void touchOff( ofxBlob& b );

    void addNewZone();
    void drawTmpZone();
    void clearZones();

    void getBackground();

	ofxKinect kinect;
    ofxBlobTracker touchTracker;

    ofFloatPixels background;
    ofxCvGrayscaleImage backgroundTex;
    ofFloatPixels current;
    // for KinectBlobTracker
    ofxCvGrayscaleImage thresMask;
    ofxCvGrayscaleImage diffMask;

    float nearThreshold;
    float farThreshold;

    float touchDiffFarThreshold;
    float touchDiffNearThreshold;

	bool bLearnBackground;

	int backFrames;

	int angle;
	int numPixels;

	unsigned int minBlobPoints;
	unsigned int maxBlobPoints;
    unsigned int maxBlobs;

    ofxCvGrayscaleImage zones;
    ofFbo fboZones;
    unsigned char zoneID;

    int t;

    vector<ofPoint> marks;
    bool marksOpen;

    ofSoundPlayer  sounds[SOUNDS];

    bool ready;
};
