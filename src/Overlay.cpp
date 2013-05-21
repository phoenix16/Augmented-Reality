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

    cv::setOpenGlDrawCallback(windowName, OverlayDrawCallback, this);

}

// Destructor
Overlay::~Overlay()
{
    cout << "Overlay::destructor" << endl;
    cv::setOpenGlDrawCallback(m_windowName, 0, 0);
}

// Public Setter function
void Overlay::setMarkerFound(const bool markerState)
{ 
    cout << "Overlay::setMarkerFound" << endl;
    markerFound = markerState;
}

// Public Setter function
void Overlay::setMarkerPose(const cv::Mat pose)
{ 
    cout << "Overlay::setMarkerPose" << endl;
    markerPose = pose;
}

// Public function
void Overlay::updateBackground(const cv::Mat& frame)
{
    cout << "Overlay::updateBackground" << endl;
    frame.copyTo(m_backgroundImage);
}

// Public function
void Overlay::updateWindow()
{
    cout << "Overlay::updateWindow" << endl;
    cv::updateWindow(m_windowName);
}

// Private function
void Overlay::draw()
{    
    cout << "Overlay::draw()" << endl;
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear entire screen:
    drawCameraFrame();                                  // Render background texture = current frame
    drawAugmentedScene();                               // Draw AR
    glFlush();
}

// Private function to set the current frame as the texture of the background
void Overlay::drawCameraFrame()
{
    cout << "Overlay::drawCameraFrame" << endl;
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
        drawCubeModel();

    }

    cout << "Overlay::drawAugmentedScene END" << endl;
}

// Private function
void Overlay::buildProjectionMatrix(const CameraCalibration& calibration, int screen_width, int screen_height, Mat44& projectionMatrix)
{
    cout << "Overlay::buildProjectionMatrix" << endl;
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
void Overlay::drawCoordinateAxis()
{
    cout << "Overlay::drawCoordinateAxis" << endl;
      glTranslatef(0.0f, 0.0f, 10.0f);  //Move forward 10 units in Z direction

    static float lineX[] = {0,0,0,1,0,0};
    static float lineY[] = {0,0,0,0,1,0};
    static float lineZ[] = {0,0,0,0,0,1};

    glLineWidth(2);

    glBegin(GL_LINES);

    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3fv(lineX);
    glVertex3fv(lineX + 3);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3fv(lineY);
    glVertex3fv(lineY + 3);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3fv(lineZ);
    glVertex3fv(lineZ + 3);

    glEnd();
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
//    glDisable(GL_TEXTURE_2D);

//    glutSwapBuffers();


//    static const GLfloat LightAmbient[]=  { 0.25f, 0.25f, 0.25f, 1.0f };    // Ambient Light Values
//    static const GLfloat LightDiffuse[]=  { 0.1f, 0.1f, 0.1f, 1.0f };    // Diffuse Light Values
//    static const GLfloat LightPosition[]= { 0.0f, 0.0f, 2.0f, 1.0f };    // Light Position

//    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT);

//    glColor4f(0.2f,0.35f,0.3f,0.75f);         // Full Brightness, 50% Alpha
//    glBlendFunc(GL_ONE,GL_ONE_MINUS_SRC_ALPHA);       // Blending Function For Translucency Based On Source Alpha
//    glEnable(GL_BLEND);

//    glShadeModel(GL_SMOOTH);

//    glEnable(GL_LIGHTING);
//    glDisable(GL_LIGHT0);
//    glEnable(GL_LIGHT1);
//    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
//    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
//    glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
//    glEnable(GL_COLOR_MATERIAL);

    // original
//    glScalef(0.25, 0.25, 0.25);
//    glTranslatef(0, 0, 1);

//    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);;
//    glRotatef(-angle, 1.0f, 1.0f, 0.0f);

//    glBegin(GL_QUADS);

//    // Front Face
//    glNormal3f( 0.0f, 0.0f, 1.0f);    // Normal Pointing Towards Viewer
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 1 (Front)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 2 (Front)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Front)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 4 (Front)

//    // Back Face
//    glNormal3f( 0.0f, 0.0f,-1.0f);    // Normal Pointing Away From Viewer
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Back)
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 2 (Back)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 3 (Back)
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 4 (Back)

//    // Top Face
//    glNormal3f( 0.0f, 1.0f, 0.0f);    // Normal Pointing Up
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 1 (Top)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 2 (Top)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Top)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 4 (Top)

//    // Bottom Face
//    glNormal3f( 0.0f,-1.0f, 0.0f);    // Normal Pointing Down
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Bottom)
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 2 (Bottom)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 3 (Bottom)
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 4 (Bottom)

//    // Right face
//    glNormal3f( 1.0f, 0.0f, 0.0f);    // Normal Pointing Right
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 1 (Right)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 2 (Right)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Right)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 4 (Right)

//    // Left Face
//    glNormal3f(-1.0f, 0.0f, 0.0f);    // Normal Pointing Left
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Left)
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 2 (Left)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 3 (Left)
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 4 (Left)
//    glEnd();

//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    glColor4f(0.2f,0.65f,0.3f,0.35f); // Full Brightness, 50% Alpha

//    glBegin(GL_QUADS);
//    // Front Face
//    glNormal3f( 0.0f, 0.0f, 1.0f);    // Normal Pointing Towards Viewer
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 1 (Front)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 2 (Front)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Front)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 4 (Front)

//    // Back Face
//    glNormal3f( 0.0f, 0.0f,-1.0f);    // Normal Pointing Away From Viewer
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Back)
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 2 (Back)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 3 (Back)
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 4 (Back)

//    // Top Face
//    glNormal3f( 0.0f, 1.0f, 0.0f);    // Normal Pointing Up
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 1 (Top)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 2 (Top)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Top)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 4 (Top)

//    // Bottom Face
//    glNormal3f( 0.0f,-1.0f, 0.0f);    // Normal Pointing Down
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Bottom)
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 2 (Bottom)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 3 (Bottom)
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 4 (Bottom)

//    // Right face
//    glNormal3f( 1.0f, 0.0f, 0.0f);    // Normal Pointing Right
//    glVertex3f( 1.0f, -1.0f, -1.0f);  // Point 1 (Right)
//    glVertex3f( 1.0f,  1.0f, -1.0f);  // Point 2 (Right)
//    glVertex3f( 1.0f,  1.0f,  1.0f);  // Point 3 (Right)
//    glVertex3f( 1.0f, -1.0f,  1.0f);  // Point 4 (Right)

//    // Left Face
//    glNormal3f(-1.0f, 0.0f, 0.0f);    // Normal Pointing Left
//    glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Left)
//    glVertex3f(-1.0f, -1.0f,  1.0f);  // Point 2 (Left)
//    glVertex3f(-1.0f,  1.0f,  1.0f);  // Point 3 (Left)
//    glVertex3f(-1.0f,  1.0f, -1.0f);  // Point 4 (Left)
//    glEnd();

//    glPopAttrib();
}




