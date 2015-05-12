#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
   // ofSetLogLevel(OF_LOG_VERBOSE);
    //----------------------shadershere *****************************************
   
    
    srcImg.loadImage("A.jpg");
    dstImg.loadImage("steamywindow.png");
    brushImg.loadImage("brush.png");
    
    trainview.loadMovie("trainview.mp4");
    trainview.play();
    
    width = ofGetWidth();
    height = ofGetHeight();
    
   
    
    
    
//---------shaders start
    
    maskFbo.allocate(width,height);
    fbo.allocate(width,height);
    
    // There are 3 of ways of loading a shader:
    //
    //  1 - Using just the name of the shader and ledding ofShader look for .frag and .vert:
    //      Ex.: shader.load( "myShader");
    //
    //  2 - Giving the right file names for each one:
    //      Ex.: shader.load( "myShader.vert","myShader.frag");
    //
    //  3 - And the third one it�s passing the shader programa on a single string;
    //
    
    
#ifdef TARGET_OPENGLES
    shader.load("shaders_gles/alphamask.vert","shaders_gles/alphamask.frag");
#else
    if(ofGetGLProgrammableRenderer()){
        string vertex = "#version 150\n\
        \n\
        uniform mat4 projectionMatrix;\n\
        uniform mat4 modelViewMatrix;\n\
        uniform mat4 modelViewProjectionMatrix;\n\
        \n\
        \n\
        in vec4  position;\n\
        in vec2  texcoord;\n\
        \n\
        out vec2 texCoordVarying;\n\
        \n\
        void main()\n\
        {\n\
        texCoordVarying = texcoord;\
        gl_Position = modelViewProjectionMatrix * position;\n\
        }";
        string fragment = "#version 150\n\
        \n\
        uniform sampler2DRect tex0;\
        uniform sampler2DRect maskTex;\
        in vec2 texCoordVarying;\n\
        \
        out vec4 fragColor;\n\
        void main (void){\
        vec2 pos = texCoordVarying;\
        \
        vec3 src = texture(tex0, pos).rgb;\
        float mask = texture(maskTex, pos).r;\
        \
        fragColor = vec4( src , mask);\
        }";
        shader.setupShaderFromSource(GL_VERTEX_SHADER, vertex);
        shader.setupShaderFromSource(GL_FRAGMENT_SHADER, fragment);
        shader.bindDefaults();
        shader.linkProgram();
    }else{
        string shaderProgram = "#version 120\n \
        #extension GL_ARB_texture_rectangle : enable\n \
        \
        uniform sampler2DRect tex0;\
        uniform sampler2DRect maskTex;\
        \
        void main (void){\
        vec2 pos = gl_TexCoord[0].st;\
        \
        vec3 src = texture2DRect(tex0, pos).rgb;\
        float mask = texture2DRect(maskTex, pos).r;\
        \
        gl_FragColor = vec4( src , mask);\
        }";
        shader.setupShaderFromSource(GL_FRAGMENT_SHADER, shaderProgram);
        shader.linkProgram();
    }
#endif
    
    // Let�s clear the FBO�s
    // otherwise it will bring some junk with it from the memory    
    maskFbo.begin();
    ofClear(125,125,125,255);
    maskFbo.end();
    
    fbo.begin();
    ofClear(125,125,125,255);
    fbo.end();
    
    bBrushDown = false;

    
//----------------------shaders end here *****************************************
    
    
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();

    
    kinect.open();		// opens first available kinect

    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
//        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
//        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
//        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
//        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    

    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 229;
    farThreshold = 225;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    trainview.update();
    
    
    
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    }
    
    blobTracker.update(grayImage, 80);
    


    
//    for (int i = 0; i < blobTracker.size(); i++){
//
//
//        if (blobTracker[i].gotFingers) {
//            bBrushDown = true;
//       
//            blobPosx = blobTracker[i].centroid.x * 1280 ;
//            blobPosy = blobTracker[i].centroid.y * 1400 - 200;
//            
//         
//            
//            brushImg.draw(blobPosx, blobPosy, 400 * blobTracker[i].boundingRect.width,400 * blobTracker[i].boundingRect.height);
//            
//            cout<< blobPosx <<endl;
//            cout<< blobPosy <<endl;
//            
//          
//  
//        }else{bBrushDown = false;}
//        
//        if (blobTracker[i].gotFingers == false){
//            bBrushDown =false;
//        }
//        
//        
//    }
//    
    
    
    
    maskFbo.begin();
    
    for (int i = 0; i < blobTracker.size(); i++){
    
    
    
    
        if (blobTracker[i].gotFingers){
                       float  blobPosx = blobTracker[i].centroid.x * 700 ;
                       float  blobPosy = blobTracker[i].centroid.y * ofGetHeight();
            
            
            brushImg.draw(blobPosx, blobPosy, 400 * blobTracker[i].boundingRect.width,400 * blobTracker[i].boundingRect.height);
            
            
            fingerReady = true;
            }else{
//                fingerReady = false;
//                int time = ofGetElapsedTimef()*1000;
//                int timeModular = time%10;
//                cout<<timeModular<<endl;
//                if(timeModular == 0){
//                    
//                    ofSetColor(0, 5);
//                    ofRect(0, 0, width, height);
//                    ofSetColor(255, 255);
//            }
        }
    }
    
    
    int time = ofGetElapsedTimef()*1000;
    int timeModular = time%27;
    cout<<timeModular<<endl;
    if(timeModular == 0){
        
        ofSetColor(0, 5);
        ofRect(0, 0, ofGetWidth(), ofGetHeight());
        ofSetColor(255, 255);
    }
    
    
    
    
    
    
    maskFbo.end();
    
    // HERE the shader-masking happends
    //
    fbo.begin();
    // Cleaning everthing with alpha mask on 0 in order to make it transparent for default
    ofClear(0, 0, 0, 0);
    
    shader.begin();
    shader.setUniformTexture("maskTex", maskFbo.getTextureReference(), 1 );
    
    trainview.draw(0, 0, ofGetWidth()+200, ofGetHeight()+200);
    //srcImg.draw(0,0);
    
    shader.end();
    fbo.end();
    

    
    
    
    
   

    
    
    
}

//--------------------------------------------------------------
void ofApp::draw() {
    ofSetColor(255,255);
    
    dstImg.draw(0,0,ofGetWidth(), ofGetHeight());
    
    fbo.draw(0,0,ofGetWidth(), ofGetHeight());
    
    
    
     grayImage.draw(10, 320, 400, 300);
     contourFinder.draw(10, 320, 400, 300);
    
    
    


    
    
    
}



//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
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
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{  bBrushDown = true;}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{  bBrushDown = false;}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
