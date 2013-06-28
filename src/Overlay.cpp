/*
 * Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
*/

#include "Overlay.h"

// Friend function that updates draw callback
void OverlayDrawCallback(void* param)
{
    Overlay *ctx = static_cast<Overlay*>(param);
    if (ctx)
    {
        cout << "Draw()" << endl;
        ctx->draw();
    }
}

// Constructor
Overlay::Overlay(std::string windowName, cv::Size windowSize, const CameraCalibration& c)
    : m_isTextureInitialized(false),
      cameraCalib(c),
      m_windowName(windowName)
{
    cout << "Overlay::constructor " << endl;

    // Create window with OpenGL support
    cv::namedWindow(windowName, cv::WINDOW_OPENGL);

    // Resize it exactly to video size
    cv::resizeWindow(windowName, windowSize.width, windowSize.height);

    // Initialize OpenGL draw callback:
    cv::setOpenGlContext(windowName);

    char *myargv [1];
    int myargc=1;
    myargv [0]=strdup ("AR");
    glutInit(&myargc, myargv);

    cv::setOpenGlDrawCallback(windowName, OverlayDrawCallback, this);

}

// Destructor
Overlay::~Overlay()
{
    cv::setOpenGlDrawCallback(m_windowName, 0, 0);
}

// Public Setter function
void Overlay::setMarkerFound(const bool markerState)
{ 
    markerFound = markerState;
}

// Public Setter function
void Overlay::setMarkerPose(const cv::Mat pose)
{ 
    markerPose = pose;
}

// Public function
void Overlay::updateBackground(const cv::Mat& frame)
{
    frame.copyTo(m_backgroundImage);
}

// Public function
void Overlay::updateWindow()
{
    cv::updateWindow(m_windowName);
}

// Private function
void Overlay::draw()
{    
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear entire screen:
    drawCameraFrame();                                  // Render background texture = current frame
    drawAugmentedScene();                               // Draw AR
    glFlush();
}

// Private function to set the current frame as the texture of the background
void Overlay::drawCameraFrame()
{
    // Initialize texture for background image
    if (!m_isTextureInitialized)
    {
        glGenTextures(1, &m_backgroundTextureId);
        glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        m_isTextureInitialized = true;
    }

    int w = m_backgroundImage.cols;
    int h = m_backgroundImage.rows;

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

    // Upload new texture data:
    if (m_backgroundImage.channels() == 3)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_backgroundImage.data);
    else if(m_backgroundImage.channels() == 4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, m_backgroundImage.data);
    else if (m_backgroundImage.channels()==1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_backgroundImage.data);

    const GLfloat bgTextureVertices[] = { 0, 0, w, 0, 0, h, w, h };
    const GLfloat bgTextureCoords[]   = { 1, 0, 1, 1, 0, 0, 0, 1 };
    const GLfloat proj[]              = { 0, -2.f/w, 0, 0, -2.f/h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

    glMatrixMode(GL_PROJECTION); // indicates next 2 lines affect projection matrix
    glLoadMatrixf(proj);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();     // reset projection matrix to original state

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

    // Update attribute values.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
    glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

    glColor4f(1,1,1,1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);
}

// Private function
// This function acts on the marker pose set by an external function
void Overlay::drawAugmentedScene()
{
    // Init augmentation projection
    Mat44 projectionMatrix;
    int w = m_backgroundImage.cols;
    int h = m_backgroundImage.rows;
    buildProjectionMatrix(cameraCalib, w, h, projectionMatrix);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projectionMatrix.data);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (markerFound)
    {
        // Set the pattern transformation
        Mat44 glMatrix;

        // Convert markerPose from a 3x3 Mat to Mat44
        // pose4x4 = pose3x3   [0]3x1
        //	         [0]1x3     1
        for (int row=0; row<4; row++)
            for (int col=0; col<4; col++)
                glMatrix.mat[row][col] = row == col ? 1 : 0;

        for (int row=0; row<3; row++)
            for (int col=0; col<3; col++)
                glMatrix.mat[row][col] = markerPose.at<float>(row, col);

        // Load the Model View matrix with marker's current pose matrix
        glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));

        // Render model
        //        drawCoordinateAxis();
        //        drawCubeModel();
        drawTeapot();

    }
}

// Private function
void Overlay::buildProjectionMatrix(const CameraCalibration& calibration, int screen_width, int screen_height, Mat44& projectionMatrix)
{
    //    float nearPlane = 0.01f;  // Near clipping distance
    //    float farPlane  = 100.0f;  // Far clipping distance

    float nearPlane = 0.1f;  // Near clipping distance
    float farPlane  = 1000.0f;  // Far clipping distance

    // Camera parameters
    float f_x = calibration.getfx(); // Focal length in x axis
    float f_y = calibration.getfy(); // Focal length in y axis (usually the same?)
    float c_x = calibration.getcx(); // Camera primary point x
    float c_y = calibration.getcy(); // Camera primary point y

    projectionMatrix.data[0] = -2.0f * f_x / screen_width;
    projectionMatrix.data[1] = 0.0f;
    projectionMatrix.data[2] = 0.0f;
    projectionMatrix.data[3] = 0.0f;

    projectionMatrix.data[4] = 0.0f;
    projectionMatrix.data[5] = 2.0f * f_y / screen_height;
    projectionMatrix.data[6] = 0.0f;
    projectionMatrix.data[7] = 0.0f;

    projectionMatrix.data[8] = 2.0f * c_x / screen_width - 1.0f;
    projectionMatrix.data[9] = 2.0f * c_y / screen_height - 1.0f;
    projectionMatrix.data[10] = -( farPlane + nearPlane) / ( farPlane - nearPlane );
    projectionMatrix.data[11] = -1.0f;

    projectionMatrix.data[12] = 0.0f;
    projectionMatrix.data[13] = 0.0f;
    projectionMatrix.data[14] = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );
    projectionMatrix.data[15] = 0.0f;

}

// Private function
void Overlay::drawTeapot()
{
    glTranslatef(0.0f, 10.0f, -8.0f);  // camera on boxes
    glColor3f (0.1, 0.1, 0.5); // color of teapot
    glutWireTeapot(1.2);
    //    glutWireTorus(0.8, 1.5, 50, 25);
    //    glutSolidCone(0.9, 2.8, 20, 50);
}


// Private function
void Overlay::drawCubeModel()
{
    float angle = 45;
    const float BOX_SIZE = 1.0f; //The length of each side of the cube

    glTranslatef(0.0f, 10.0f, -8.0f);  // camera on boxes

    GLfloat ambientLight[] = {0.3f, 0.3f, 0.3f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);

    GLfloat lightColor[] = {0.7f, 0.7f, 0.7f, 1.0f};
    GLfloat lightPos[] = {-2 * BOX_SIZE, BOX_SIZE, 4 * BOX_SIZE, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glRotatef(-angle, 1.0f, 1.0f, 0.0f);

    glBegin(GL_QUADS);

    //Top face
    glColor3f(1.0f, 1.0f, 0.0f);
    glNormal3f(0.0, 1.0f, 0.0f);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);

    //Bottom face
    glColor3f(1.0f, 0.0f, 1.0f);
    glNormal3f(0.0, -1.0f, 0.0f);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);

    //Left face
    glNormal3f(-1.0, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);

    //Right face
    glNormal3f(1.0, 0.0f, 0.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);

    //Front face
    glNormal3f(0.0, 0.0f, 1.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);

    //Back face
    glNormal3f(0.0, 0.0f, -1.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);

    glEnd();
}




