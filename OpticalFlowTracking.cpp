
int main(int argc, char** argv) 
{
    initGL(argc,argv);
    initOCV(NULL);
     
    pthread_t tId;
    pthread_attr_t tAttr;
    pthread_attr_init(&tAttr);
    pthread_create(&tId, &tAttr, startOCV, NULL);
 
    startGL(NULL);  
}


void* startOCV(void* arg) 
{
    while (1) 
    {
        cvtColor(img, prev, CV_BGR2GRAY);
 
        //get frame off camera
        cap >> frame;
        if(frame.data == NULL) break;
         
        frame.copyTo(img);
 
        cvtColor(img, next, CV_BGR2GRAY);
 
        //calc optical flow     
        calcOpticalFlowPyrLK(prev, next, points1, points2, status, err, Size(30,30));
        cvtPtoKpts(imgPointsOnPlane, points2);
 
        //switch points vectors (next becomes previous)
        points1.clear();
        points1 = points2;
         
        //calculate camera pose
        getPlanarSurface(points1);
 
        //refresh 3D scene
        glutPostWindowRedisplay(glutwin);
         
        //show tracked points on scene
        drawKeypoints(next, imgPointsOnPlane, img_to_show, Scalar(255));
        imshow("main2", img_to_show);
        int c = waitKey(30);
        if (c == ' ') {
            waitKey(0);
        }
    }
    return NULL;
}

void getPlanarSurface(vector<Point2f>& imgP)
{    
    Rodrigues(rotM,rvec);
     
    solvePnP(objPM, Mat(imgP), camera_matrix, distortion_coefficients, rvec, tvec, true);
     
    Rodrigues(rvec,rotM);
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
    //Make sure we have a background image buffer
    if(img_to_show.data != NULL) {
        Mat tmp; 
         
        //Switch to Ortho for drawing background
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluOrtho2D(0.0, 0.0, 640.0, 480.0);
         
        glMatrixMode(GL_MODELVIEW);
         
        //Textures can only have power-of-two dimensions, so closest to 640x480 is 1024x512
        tmp = Mat(Size(1024,512),CV_8UC3);
        //However we are going to use only a portion, so create an ROI
        Mat ttmp = tmp(Range(0,img_to_show.rows),Range(0,img_to_show.cols));
 
        //Some frames could be 8bit grayscale, so make sure on the output we always get 24bit RGB.
        if(img_to_show.step == img_to_show.cols)
            cvtColor(img_to_show, ttmp, CV_GRAY2RGB);
        else if(img_to_show.step == img_to_show.cols * 3)
            cvtColor(img_to_show, ttmp, CV_BGR2RGB);
        flip(ttmp,ttmp,0);
         
        glEnable(GL_TEXTURE_2D);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, 1024, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, tmp.data);
         
        //Finally, draw the texture using a simple quad with texture coords in corners.
        glPushMatrix();
        glTranslated(-320.0, -240.0, -500.0);//why these parameters?!
        glBegin(GL_QUADS);
        glTexCoord2i(0, 0); glVertex2i(0, 0);
        glTexCoord2i(1, 0); glVertex2i(640, 0);
        glTexCoord2i(1, 1); glVertex2i(640, 480);
        glTexCoord2i(0, 1); glVertex2i(0, 480);
        glEnd();
        glPopMatrix();
 
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
     
    glPushMatrix();
    double m[16] = {    _d[0],-_d[3],-_d[6],0,
                        _d[1],-_d[4],-_d[7],0,
                        _d[2],-_d[5],-_d[8],0,
                        tv[0],-tv[1],-tv[2],1   };
     
    //Rotate and translate according to result from solvePnP
    glLoadMatrixd(m);
     
    //Draw a basic cube
    glDisable(GL_TEXTURE_2D);
    glColor3b(255, 0, 0);
    glutSolidCube(1);
    glPopMatrix();
 
    glutSwapBuffers();
}