#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBlobTracker.h"


class ofApp : public ofBaseApp {
public:
    
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawPointCloud();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofxKinect kinect;

    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCvContourFinder contourFinder;
    
    ofxBlobTracker blobTracker;
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    bool mDrawKinectDepthImage = false;
   
    
    int nearThreshold;
    int farThreshold;
    
    int angle;
    
    ofVideoPlayer trainview;
    
    
 //----------------shaders variables here______________________
    ofImage     srcImg;
    ofImage     dstImg;
    ofImage     brushImg;
    ofVideoPlayer video;
    
    int width;
    int height;
    
    ofFbo       maskFbo;
    ofFbo       fbo;
    
    ofShader    shader;
    
    bool        bBrushDown;
    bool        fingerReady;
    
//_______________________
    

};
