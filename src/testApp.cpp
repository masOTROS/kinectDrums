#include "testApp.h"
//--------------------------------------------------------------
void testApp::setup() {
    ofEnableAlphaBlending();
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=-30;
	kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	kinect.enableDepthNearValueWhite(true);

	ofAddListener(touchTracker.blobAdded, this, &testApp::touchOn);
    ofAddListener(touchTracker.blobMoved, this, &testApp::touchMoved);
    ofAddListener(touchTracker.blobDeleted, this, &testApp::touchOff);

    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width,kinect.height);//,OF_IMAGE_GRAYSCALE);
    diffMask.allocate(kinect.width, kinect.height);
    thresMask.allocate(kinect.width, kinect.height);

    nearThreshold=560.;
    farThreshold=880.;

    touchDiffFarThreshold=120.;
    touchDiffNearThreshold=7.;

    numPixels = kinect.width*kinect.height;

    maxBlobs = 10;
    minBlobPoints=0;
    maxBlobPoints=1000000;

	bLearnBackground = true;
	backFrames=0;

	marksOpen=false;
	fboZones.allocate(kinect.width,kinect.height);
	zoneID=0;
	zones.allocate(kinect.width,kinect.height);

	t=100;

    for(int i=0;i<SOUNDS;i++)
    {
        stringstream name;
        name << "sounds/" << i << ".wav";
        sounds[i].loadSound(name.str());
        sounds[i].setVolume(0.75f);
        sounds[i].setMultiPlay(true);
    }
    ready=false;

	ofSetFrameRate(60);

}

//--------------------------------------------------------------
void testApp::update() {

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load depth image from the kinect source
		current=kinect.getDistancePixelsRef();

        unsigned char * tmpDiffMask = new unsigned char[numPixels];
        unsigned char * tmpThresMask = new unsigned char[numPixels];
        unsigned char * zonesPtr = zones.getPixels();
        float diff;

        if(bLearnBackground || backFrames)
        {
            unsigned char * tmpCurrent = kinect.getDepthPixels();
            unsigned char * tmpBackground = backgroundTex.getPixels();
            if(backFrames){
                for(int i=0;i<numPixels;i++)
                {
                    if(background[i]){
                        if(current[i]){
                            background[i]=(background[i]+current[i])/2.;
                            unsigned int mean = tmpCurrent[i]+ tmpBackground[i];
                            tmpBackground[i] = (unsigned char)(mean/2);
                        }
                    }
                    else{
                        background[i]=current[i];
                        tmpBackground[i]=tmpCurrent[i];
                    }
                }
                backFrames--;
            }
            if(bLearnBackground){
                for(int i=0;i<numPixels;i++)
                {
                    background[i]=current[i];
                    tmpBackground[i]= tmpCurrent[i];
                }
                backFrames=BACKGROUND_FRAMES;
                bLearnBackground = false;
            }
            backgroundTex.flagImageChanged();
        }

        for(int i=0;i<numPixels;i++)
        {
            tmpDiffMask[i]=0;
            tmpThresMask[i]=0;
            if(current[i]<farThreshold && current[i]>nearThreshold)
            {
                tmpThresMask[i]=(unsigned char)ofMap(current[i],nearThreshold,farThreshold,255,0);
                if(zonesPtr[i]){
                    diff=background[i]-current[i];
                    if(diff>touchDiffNearThreshold && diff<touchDiffFarThreshold)
                    {
                        tmpDiffMask[i]=255;//(unsigned char)ofMap(diff,touchDiffNearThreshold,touchDiffFarThreshold,100,255);
                    }
                }
            }
        }
        thresMask.setFromPixels(tmpThresMask, kinect.width, kinect.height);
        diffMask.setFromPixels(tmpDiffMask, kinect.width, kinect.height);

        cvErode(diffMask.getCvImage(), diffMask.getCvImage(), NULL, 4);
        cvDilate(diffMask.getCvImage(), diffMask.getCvImage(), NULL, 10);

        touchTracker.update(diffMask,-1, minBlobPoints , maxBlobPoints, maxBlobs,20,false, true);
    }

    ofSoundUpdate();
}

//--------------------------------------------------------------
void testApp::draw() {
    if(!ready){
    ofBackground(100, 100, 100);

	ofSetColor(255, 255, 255);
    // draw from the live kinect
    kinect.drawDepth(10, 10, 320, 240);

    backgroundTex.draw(340,10,320,240);

    thresMask.draw(10, 260, 320, 240);

    diffMask.draw(340, 260, 320, 240);

    ofSetColor(255,255,0,100);
    fboZones.draw(10,260,320,240);

    if(t>100)
        t--;
    ofSetColor(50,50,255,t);
    zones.draw(340,260,320,240);

    touchTracker.draw(340, 260, 320, 240);

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "press b to set current frame as background" << endl
	<< "num touch points found: " << touchTracker.size()
	<< ", fps: " << ofToString(ofGetFrameRate(),2) << endl
	<< "press n and m to change touchDiffFarThreshold:"<< ofToString(touchDiffFarThreshold,2) << "   k and l to change touchDiffNearThreshold:"<< ofToString(touchDiffNearThreshold,2) << endl
	<< "press w and s to change farThreshold:"<< farThreshold << "   e and d to change nearThreshold:"<< nearThreshold << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,510);
	if(backFrames)
	{
        ofDrawBitmapString("WWWAIT!",360,100);
    }
    if(marksOpen)
    {
        ofDrawBitmapString("Ready to add points",10,270);
    }
    }
}

void testApp::exit() {
    kinect.setCameraTiltAngle(0);
	kinect.close();
}
//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
	    case ' ':
            ready = !ready;
            break;

		case 'b':
            bLearnBackground = true;
            break;

		case 'a':
			marksOpen=!marksOpen;
			if(!marksOpen)
                addNewZone();
            break;

        case 'D':
			clearZones();
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;

        case 'w':
            farThreshold+=20.; //5cm
            break;
        case 's':
            farThreshold-=20.; //5cm
            break;
        case 'e':
            nearThreshold+=20.; //5cm
            break;

        case 'd':
            nearThreshold-=20.; //5cm
            break;

        case 'n':
			touchDiffFarThreshold-=0.5;
			break;
       case 'm':
			touchDiffFarThreshold+=0.5;
			break;
        case 'k':
			touchDiffNearThreshold-=0.5;
			break;
        case 'l':
			touchDiffNearThreshold+=0.5;
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    if(mouseX>10 && mouseX<340 && mouseY>260 && mouseY<500){
        if(marksOpen){
            marks.push_back(ofPoint(2*(mouseX-10),2*(mouseY-260)));
            drawTmpZone();
        }
    }
}

void testApp::drawTmpZone()
{
    if(marks.size()==1)
    {
        fboZones.begin();
        ofClear(0,0);
        ofSetColor(255);
        zones.draw(0,0);
        ofSetColor(255-zoneID);
        ofPoint p0=marks.front();
        ofLine(p0.x,p0.y,p0.x,p0.y);
        fboZones.end();

    }
    if(marks.size()==2)
    {
        fboZones.begin();
        ofClear(0,0);
        ofSetColor(255);
        zones.draw(0,0);
        ofSetColor(255-zoneID);
        ofPoint p0=marks.front();
        ofPoint p1=marks.back();
        ofLine(p0.x,p0.y,p1.x,p1.y);
        fboZones.end();

    }
    if(marks.size()>=3)
    {
        fboZones.begin();
        ofClear(0,0);
        ofSetColor(255);
        zones.draw(0,0);
        ofSetColor(255-zoneID);
        ofPoint p0=marks.front();
        ofBeginShape();
        ofCurveVertex(p0.x,p0.y);
        ofCurveVertex(p0.x,p0.y);
        for(int i=1;i<(marks.size()-1);i++)
        {
            ofPoint p=marks[i];
            ofCurveVertex(p.x,p.y);
        }
        ofPoint p1=marks.back();
        ofCurveVertex(p1.x,p1.y);
        ofCurveVertex(p1.x,p1.y);
        ofEndShape();
        fboZones.end();
    }
}

void testApp::addNewZone()
{
    zoneID=(zoneID+1)%ZONES;
    marks.clear();

    ofPixels tmpPixColor;
    ofPixels tmpPixGray;
    fboZones.readToPixels(tmpPixColor);
    tmpPixGray.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    for(int i=0;i<numPixels;i++)
    {
        tmpPixGray[i]=tmpPixColor[i*4];
    }
    zones.setFromPixels(tmpPixGray);
}
void testApp::clearZones()
{
    zones.clear();
    zones.allocate(kinect.width,kinect.height);
    fboZones.begin();
    ofClear(0,0);
    fboZones.end();
    zoneID=255;
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
/*
 *
 *	blob section
 *
 *	from here on in it's blobs
 *	thanks to stefanix and the opencv library :)
 *
 */

//--------------------------------------------------
void testApp::touchOn( ofxBlob& b ) {
    int x=b.centroid.x*kinect.width;
    int y=b.centroid.y*kinect.height;
    unsigned char * zonesPtr=zones.getPixels();
    if(zonesPtr[x+y*kinect.width])
    {
        t=255;
        int ID=(255-zonesPtr[x+y*kinect.width]);
        sounds[ID].play();
    }
}

void testApp::touchMoved( ofxBlob& b ) {
    //cout << "blobMoved() - id:" << id << " order:" << order << endl;
  // full access to blob object ( get a reference)
//    ofxKinectTrackedBlob blob = blobTracker.getById( id );
 //  cout << "volume: " << blob.volume << endl;
}

void testApp::touchOff( ofxBlob& b ) {
   //cout << "blobOff() - id:" << id << " order:" << order << endl;
}
