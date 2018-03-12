#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <opencv2/core/core.hpp>// OpenCV Header
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <OpenNI.h>// OpenNI Header
#include <NiTE.h> // nite Header
#include <cstdio>
#include <string>
#include <Windows.h>
#include <wingdi.h>
#include <math.h>
#include <GL/gl.h>
#include <glut.h>
#include"myMask.h"//mask face



using namespace std;// namespace
using namespace openni;
using namespace nite;

cv::VideoCapture *cap = NULL;
int width = 640;
int height = 480;
cv::Mat image;
VideoStream mColorStream;
VideoStream mDepthStream;

//NITE global
UserTracker mUserTracker;
//check 
bool flag_move = false, flag_zoom = false;
bool flag_changemodel2=false,flag_changemodelcheck=false;
bool flag_first=true;
int changemodel_count=0;

float deArray[10][640 * 480];
int depth_count = 0, depth_start = 0;
cv::Mat coordinateTransform(cv::Mat source);


float offsetx = 0, offsety = 0;
float temprighthandx, temprighthandy;
float templefthandx, templefthandy;
float initx1 = -3.1, initx2 = 3.1, inity1 = 3.1, inity2 = -3.1;// initx1:left x point  initx2:right x point  inity1:up y point inity2:down y point
int realworldx = 0, realworldy = 0;//initial realworld x,y

//ball
float light_position[] = { -20, 20, 0 };               //��������m
unsigned int textures[6];                        //�Ψ��x�s6�i���誫�󪺽s��
float priorities[6] = { 1.0, 0.8, 0.6, 0.4, 0.2, 0.0 };         //�R�����誫�󪺶���
GLuint ballModel; //�y���ҫ����
GLUquadric *ball;
float ball_translatex = 0, ball_translatey = 0;
float  radius=4.5;


void WindowSize(int, int);                     //�t�d������ø�Ϥ��e�����
void Keyboard(unsigned char, int, int);            //�����L��J
void Mouse(int, int, int, int);                  //����ƹ����U�M��}�ɪ��T��
void MotionMouse(int, int);                     //����ƹ����U�������T��
void Display(void);                              //�yø

void SetLightSource(void);                        //�]�w�����ݩ�
void SetMaterial(void);                           //�]�w�����ݩ�
void Texture(void);                              //�B�z����K�Ϫ��������O
unsigned char *LoadBitmapFile(char *, BITMAPINFO *);   //�ΨӱNBMP����Ū�J�çאּRGB�榡
void SetTexObj(char *, int);                     //�N�Ϥ���J���誫�󤤡A�ĤG�ӰѼƯ���w���誫��
void loadObj(char *fname);						// Ū��model�ɮ� (obj��)
void drawBall();								//�emodel
void drawBall2();


void display()
{
	//depth
	mDepthStream.start();
	VideoFrameRef frameDepth;
	mDepthStream.readFrame(&frameDepth);
	const openni::DepthPixel* pDepth
		= (const openni::DepthPixel*)frameDepth.getData();
	int idx = frameDepth.getWidth() * (frameDepth.getHeight() + 1) / 2;
	int count = 0;
	//
	float* DepthArray = new float[640 * 480];

	//total frame don't reach to 5
	if (depth_start<5)
	{
		for (int y = 0; y < frameDepth.getHeight(); ++y)
		{
			for (int x = 0; x < frameDepth.getWidth(); ++x)
			{
				int idx = x + y * frameDepth.getWidth();
				deArray[depth_start][count] = pDepth[idx];
				//compensate error
				if(pDepth[idx]>5000 || pDepth[idx]==0)
				{
					deArray[depth_start][count]=3000;
				}
				count++;
			}
		}
		depth_start++;

		for (int xx = 0; xx<640 * 480; xx++)
		{
			int temp = 0;
			for (int yy = 0; yy<depth_start; yy++)
			{
				temp += deArray[yy][xx];
			}
			DepthArray[xx] = temp / depth_start;
		}
	}
	else
	{
		//total frame more than 10 frame
		for (int y = 0; y < frameDepth.getHeight(); ++y)
		{
			for (int x = 0; x < frameDepth.getWidth(); ++x)
			{
				int idx = x + y * frameDepth.getWidth();
				deArray[depth_count][count] = pDepth[idx]; //average and weighted
				if(pDepth[idx]>5000 || pDepth[idx]==0)
				{
					deArray[depth_count][count]=3000;
				}

				count++;
			}
		}
		depth_count = (depth_count + 1) % 5;

		for (int xx = 0; xx<640 * 480; xx++)
		{
			int temp = 0;
			for (int yy = 0; yy<10; yy++)
			{
				temp += deArray[yy][xx];
			}
			DepthArray[xx] = temp / 5;
		}
	}


	int iMaxDepth = mDepthStream.getMaxPixelValue();
	if (mDepthStream.readFrame(&frameDepth) == openni::STATUS_OK)
	{
		// 8b. convert data to OpenCV format
		const cv::Mat mImageDepth(
			frameDepth.getHeight(), frameDepth.getWidth(),
			CV_16UC1, (void*)frameDepth.getData());
		// 8c. re-map depth data [0,Max] to [0,255]
		cv::Mat mScaledDepth;
		mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);
		// 8d. show image
		cv::imshow("Depth Image", mScaledDepth);
	}



	cv::Mat cImageBGR;
	VideoFrameRef  mColorFrame;
	mColorStream.start();
	if (mColorStream.isValid())
	{
		// get color frame
		if (mColorStream.readFrame(&mColorFrame) == openni::STATUS_OK)
		{
			//convert data to OpenCV format
			const cv::Mat mImageRGB(
				mColorFrame.getHeight(), mColorFrame.getWidth(),
				CV_8UC3, (void*)mColorFrame.getData());
			//convert form RGB to BGR
			//cv::Mat cImageBGR;
			cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);

			//nite!!!
			// get user frame
			UserTrackerFrameRef  mUserFrame;
			mUserTracker.readFrame(&mUserFrame);

			// get users data
			const nite::Array<UserData>& aUsers = mUserFrame.getUsers();
			for (int i = 0; i < aUsers.getSize(); ++i)
			{
				const UserData& rUser = aUsers[i];

				//check user status
				if (rUser.isNew())
				{
					// start tracking for new user
					mUserTracker.startSkeletonTracking(rUser.getId());
				}

				if (rUser.isVisible())
				{
					// get user skeleton
					const Skeleton& rSkeleton = rUser.getSkeleton();
					if (rSkeleton.getState() == SKELETON_TRACKED)
					{
						// build joints array
						SkeletonJoint aJoints[15];
						aJoints[0] = rSkeleton.getJoint(JOINT_HEAD);
						aJoints[1] = rSkeleton.getJoint(JOINT_NECK);
						aJoints[2] = rSkeleton.getJoint(JOINT_LEFT_SHOULDER);
						aJoints[3] = rSkeleton.getJoint(JOINT_RIGHT_SHOULDER);
						aJoints[4] = rSkeleton.getJoint(JOINT_LEFT_ELBOW);
						aJoints[5] = rSkeleton.getJoint(JOINT_RIGHT_ELBOW);
						aJoints[6] = rSkeleton.getJoint(JOINT_LEFT_HAND);
						aJoints[7] = rSkeleton.getJoint(JOINT_RIGHT_HAND);
						aJoints[8] = rSkeleton.getJoint(JOINT_TORSO);
						aJoints[9] = rSkeleton.getJoint(JOINT_LEFT_HIP);
						aJoints[10] = rSkeleton.getJoint(JOINT_RIGHT_HIP);
						aJoints[11] = rSkeleton.getJoint(JOINT_LEFT_KNEE);
						aJoints[12] = rSkeleton.getJoint(JOINT_RIGHT_KNEE);
						aJoints[13] = rSkeleton.getJoint(JOINT_LEFT_FOOT);
						aJoints[14] = rSkeleton.getJoint(JOINT_RIGHT_FOOT);

						//convert joint position to image
						cv::Point2f aPoint[15];
						Point3f Point[15];
						for (int s = 0; s < 15; ++s)
						{
							const Point3f& rPos = aJoints[s].getPosition();
							mUserTracker.convertJointCoordinatesToDepth(rPos.x, rPos.y, rPos.z, &(aPoint[s].x), &(aPoint[s].y));
							Point[s].x = rPos.x;
							Point[s].y = rPos.y;
							Point[s].z = rPos.z;
						}

						//draw line
						cv::line(cImageBGR, aPoint[0], aPoint[1], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[1], aPoint[2], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[1], aPoint[3], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[2], aPoint[4], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[3], aPoint[5], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[4], aPoint[6], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[5], aPoint[7], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[1], aPoint[8], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[8], aPoint[9], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[8], aPoint[10], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[9], aPoint[11], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[10], aPoint[12], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[11], aPoint[13], cv::Scalar(255, 0, 0), 3);
						cv::line(cImageBGR, aPoint[12], aPoint[14], cv::Scalar(255, 0, 0), 3);

						//draw joint
						for (int s = 0; s < 15; ++s)
						{
							if (aJoints[s].getPositionConfidence() > 0.5)
								cv::circle(cImageBGR, aPoint[s], 3, cv::Scalar(0, 0, 255), 2);
							else
								cv::circle(cImageBGR, aPoint[s], 3, cv::Scalar(0, 255, 0), 2);
						}
						if (flag_move)
						{
							//move Center
							realworldx += (Point[7].x - temprighthandx);
							realworldy += (Point[7].y - temprighthandy);

							//move in OpenGL
							offsetx = (Point[7].x - temprighthandx) / 100;
							offsety = (Point[7].y - temprighthandy) / 100;
							ball_translatex += offsetx;
							ball_translatey += offsety;
							
							cout << offsetx << " " << offsety << endl;
						}
						if (flag_zoom)
						{
							//zoom
							float rightx = (Point[7].x - temprighthandx) / 500;
							float righty = (Point[7].y - temprighthandy) / 500;
							float leftx = (templefthandx - Point[6].x) / 500;
							float lefty = (templefthandy - Point[6].y) / 500;
							offsetx = rightx + leftx;

							radius+=offsetx;
							
							cout << offsetx << " " << offsety << endl;

						}
						cout << Point[6].x << " " << Point[6].y << endl;

						if(Point[7].x<Point[6].x && Point[5].x>Point[4].x &&Point[7].y>Point[4].y && Point[7].y>Point[5].y
								&& Point[6].y>Point[5].y && Point[6].y>Point[4].y
							    && aJoints[4].getPositionConfidence()>=0.6 &&  aJoints[5].getPositionConfidence()>=0.6
								&& aJoints[6].getPositionConfidence()>=0.6 && aJoints[7].getPositionConfidence()>=0.6)
						{
							//change model
							if(!flag_changemodelcheck && !flag_changemodel2)
							{
								flag_changemodel2=true;
								flag_changemodelcheck=true;
							}
							else if(!flag_changemodelcheck && flag_changemodel2)
							{
								flag_changemodel2=false;
								flag_changemodelcheck=true;
							}
						}
						else if (Point[7].x<(300 + realworldx+radius*5) && Point[7].x>(0 + realworldx) && Point[7].z>1000 && Point[7].z<2000  &&Point[6].z<2000
							&& Point[6].x<(0 + realworldx) && Point[6].x>(-300 + realworldx-radius*5) &&flag_first
							&& aJoints[7].getPositionConfidence()>0.6 && aJoints[6].getPositionConfidence()>0.6)
						{
							//check for zoom
							flag_zoom = true;
							flag_move = false;
							temprighthandx = Point[7].x;
							temprighthandy = Point[7].y;
							templefthandx = Point[6].x;
							templefthandy = Point[6].y;
							flag_first=false;
						}
						else if (Point[7].x<(150 + realworldx) && Point[7].x>(-150 + realworldx) && flag_first && Point[7].z<2000 && Point[7].z>1000 && aJoints[7].getPositionConfidence()>0.6)
						{
							//check for move
							flag_move = true;
							flag_zoom = false;
							temprighthandx = Point[7].x;
							temprighthandy = Point[7].y;
							flag_first=false;
							//cout<<tempx<<" "<<tempy<<endl;
						}
						else if (Point[7].x<(300 + realworldx+radius*6) && Point[7].x>(0 + realworldx) && Point[7].y<(300 + realworldy) && Point[7].y>(-300 + realworldy) && Point[7].z<2000 && Point[7].z>1000  &&Point[6].z<2000
							&& Point[6].x<(0 + realworldx) && Point[6].x>(-300 + realworldx-radius*6) && Point[6].y<(300 + realworldy) && Point[6].y>(-300 + realworldy)
							&& aJoints[7].getPositionConfidence()>0.65 && aJoints[6].getPositionConfidence()>0.65)
						{
							//check for zoom
							flag_zoom = true;
							flag_move = false;
							temprighthandx = Point[7].x;
							temprighthandy = Point[7].y;
							templefthandx = Point[6].x;
							templefthandy = Point[6].y;
							changemodel_count++;
							if(changemodel_count>=10)
							{
								flag_changemodelcheck=false;
								changemodel_count=0;
							}
						}
						else if (Point[7].x<(150 + realworldx) && Point[7].x>(-150 + realworldx) && Point[7].y<(150 + realworldy) && Point[7].y>(-100 + realworldy) && Point[7].z<2000 && Point[7].z>1000 && aJoints[7].getPositionConfidence()>0.6)
						{
							//check for move
							flag_move = true;
							flag_zoom = false;
							temprighthandx = Point[7].x;
							temprighthandy = Point[7].y;
							//cout<<tempx<<" "<<tempy<<endl;
							changemodel_count++;
							if(changemodel_count>=10)
							{
								flag_changemodelcheck=false;
								changemodel_count=0;
							}
						}
						else
						{
							flag_move = false;
							flag_zoom = false;
							//flag_changemodelcheck=false;
							changemodel_count++;
							if(changemodel_count>=10)
							{
								flag_changemodelcheck=false;
								changemodel_count=0;
							}
						}


					}
				}
			}


			//nite end!!
			//show image
			cv::imshow("Color Image", cImageBGR);
		}
	}


	

	// clear the window
	//glClear( GL_COLOR_BUFFER_BIT );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// show the current camera frame

	//based on the way cv::Mat stores data, you need to flip it before displaying it
	glDisable(GL_DEPTH_TEST);
	cv::Mat tempimage;
	cv::flip(cImageBGR, tempimage, 0);
	//glDrawPixels( tempimage.size().width, tempimage.size().height, GL_BGR, GL_UNSIGNED_BYTE, tempimage.ptr() );
	glDrawPixels(tempimage.size().width, tempimage.size().height, 0x80E0, GL_UNSIGNED_BYTE, tempimage.ptr());
	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	//draw virtual
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, tempimage.size().width*1.0 / tempimage.size().height, 1, 20);
	glDisable(GL_BLEND);
	glViewport(0, 0, tempimage.size().width, tempimage.size().height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);

	glPushMatrix();
	//move to the position where you want the 3D object to go
	glTranslatef(0, 0, 0); //this is an arbitrary position for demonstration
	//you will need to adjust your transformations to match the positions where
	//you want to draw your objects(i.e. chessboard center, chessboard corners)
	glColor3f(1.0f, 1.0f, 1.0f);
	glPopMatrix();

	
	setDepthValues(DepthArray);
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); // NO COLOR!!
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, vertexCoords);
	glDrawArrays(GL_POINTS, 0, VERTEX_NUM);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);// YES COLOR^___^
	glDisableClientState(GL_VERTEX_ARRAY);

	if(flag_move)
	{
		glEnable(GL_BLEND);
		glColor4f(1.0f, 1.0f, 0.0f, 0.5f);
		glBindTexture(GL_TEXTURE_2D, textures[0]);//��ܧA�n�Ϊ�����
		if(flag_changemodel2)
		{
			glBindTexture(GL_TEXTURE_2D, textures[1]);//��ܧA�n�Ϊ�����
			drawBall2();
		}
		else
		{
			glBindTexture(GL_TEXTURE_2D, textures[0]);//��ܧA�n�Ϊ�����
			drawBall();
		}
		glBindTexture(GL_TEXTURE_2D, textures[1]);//��ܧA�n�Ϊ�����
	}
	else if(flag_zoom)
	{
		glEnable(GL_BLEND);
		glColor4f(0.5f, 0.0f, 0.0f, 0.5f);
		glBindTexture(GL_TEXTURE_2D, textures[0]);//��ܧA�n�Ϊ�����
		if(flag_changemodel2)
		{
			glBindTexture(GL_TEXTURE_2D, textures[1]);//��ܧA�n�Ϊ�����
			drawBall2();
		}
		else
		{
			glBindTexture(GL_TEXTURE_2D, textures[0]);//��ܧA�n�Ϊ�����
			drawBall();
		}
		
	}
	else
	{
		glEnable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);
		glColor4f(1.0f, 1.0f, 0.0f, 0.5f);
		
		if(flag_changemodel2)
		{
			glBindTexture(GL_TEXTURE_2D, textures[1]);//��ܧA�n�Ϊ�����
			drawBall2();
		}
		else
		{
			glBindTexture(GL_TEXTURE_2D, textures[0]);//��ܧA�n�Ϊ�����
			drawBall();
		}
		//another ball model
		glDisable(GL_TEXTURE_2D);
		
	}
	// show the rendering on the screen
	
	glutSwapBuffers();

	// post the next redisplay
	glutPostRedisplay();
	delete DepthArray;

}

void reshape(int w, int h)
{
	// set OpenGL viewport (drawable area)
	glViewport(0, 0, w, h);
}

void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{

	}
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
		//case 'q':
		// quit when q is pressed
	default:
		cout << "heere" << endl;
		exit(0);
		break;

	}
}

void idle()
{
	// grab a frame from the camera
	// (*cap) >> image;
}

int main(int argc, char **argv)
{
	//int w,h;
	maskinitial(0, 0);

	//initial openni
	if (OpenNI::initialize() != openni::STATUS_OK)
	{
		cerr << "OpenNI Initial Error: "
			<< OpenNI::getExtendedError() << endl;
		return -1;
	}

	// 2. Open Device
	Device mDevice;
	if (mDevice.open(ANY_DEVICE) != openni::STATUS_OK)
	{
		cerr << "Can't Open Device: "
			<< OpenNI::getExtendedError() << endl;
		return -1;
	}

	if (mDevice.hasSensor(SENSOR_DEPTH))
	{
		if (mDepthStream.create(mDevice, SENSOR_DEPTH) == openni::STATUS_OK)
		{
			// 3a. set video mode
			VideoMode mMode;
			mMode.setResolution(640, 480);
			mMode.setFps(30);
			mMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);


			if (mDepthStream.setVideoMode(mMode) != openni::STATUS_OK)
			{
				cout << "Can't apply VideoMode: "
					<< OpenNI::getExtendedError() << endl;
			}
		}
		else
		{
			cerr << "Can't create depth stream on device: "
				<< OpenNI::getExtendedError() << endl;
			return -1;
		}
	}
	else
	{
		cerr << "ERROR: This device does not have depth sensor" << endl;
		return -1;
	}


	//mColorStream
	//VideoStream mColorStream;
	if (mDevice.hasSensor(SENSOR_COLOR))
	{
		if (mColorStream.create(mDevice, SENSOR_COLOR) == openni::STATUS_OK)
		{
			// 4a. set video mode
			VideoMode mMode;
			mMode.setResolution(640, 480);
			mMode.setFps(30);
			mMode.setPixelFormat(PIXEL_FORMAT_RGB888);

			if (mColorStream.setVideoMode(mMode) != openni::STATUS_OK)
			{
				cout << "Can't apply VideoMode: "
					<< OpenNI::getExtendedError() << endl;
			}

			// 4b. image registration
			if (mDevice.isImageRegistrationModeSupported(
				IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			{
				mDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
			}
		}
		else
		{
			cerr << "Can't create color stream on device: "
				<< OpenNI::getExtendedError() << endl;
			return -1;
		}
	}
	//nite initial
	NiTE::initialize();

	// create user tracker
	mUserTracker.create(&mDevice);
	mUserTracker.setSkeletonSmoothingFactor(0.5f);

	cout << "ha" << endl;
	width = 640;
	height = 480;
	cout << width << endl;
	cout << height << endl;

	// initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowPosition(20, 20);
	glutInitWindowSize(width, height);
	glutCreateWindow("OpenGL / OpenCV Example");
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	Texture(); 

	//�U�����ӬO�Ψӫ��wCallback���
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(display);

	//SetLightSource();
	//SetMaterial();
	loadObj("Debug\\objModel\\ball.txt");


	// start GUI loop
	glutMainLoop();
	return 0;
}
cv::Mat coordinateTransform(cv::Mat source)
{
	int alpha_ = 90., beta_ = 90., gamma_ = 90.;
	int f_ = 500, dist_ = 500;
	double pi = 3.1415926;
	cv::Mat destination;

	double f, dist;
	double alpha, beta, gamma;
	alpha = ((double)alpha_ - 90.)*pi / 180;
	beta = ((double)beta_ - 90.)*pi / 180;
	gamma = ((double)gamma_ - 90.)*pi / 180;
	f = (double)f_;
	dist = (double)dist_;

	cv::Size taille = source.size();
	double w = (double)taille.width, h = (double)taille.height;
	double size = w*h;
	// Projection 2D -> 3D matrix
	cv::Mat A1 = (cv::Mat_<double>(4, 3) <<
		1, 0, -w / 2,
		0, 1, -h / 2,
		0, 0, 0,
		0, 0, 1);

	// Rotation matrices around the X,Y,Z axis
	cv::Mat RX = (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha), cos(alpha), 0,
		0, 0, 0, 1);

	cv::Mat RY = (cv::Mat_<double>(4, 4) <<
		cos(beta), 0, -sin(beta), 0,
		0, 1, 0, 0,
		sin(beta), 0, cos(beta), 0,
		0, 0, 0, 1);

	cv::Mat RZ = (cv::Mat_<double>(4, 4) <<
		cos(gamma), -sin(gamma), 0, 0,
		sin(gamma), cos(gamma), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);

	// Composed rotation matrix with (RX,RY,RZ)
	cv::Mat R = RX * RY * RZ;

	// Translation matrix on the Z axis change dist will change the height
	cv::Mat T = (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, dist,
		0, 0, 0, 1);

	// Camera Intrisecs matrix 3D -> 2D
	cv::Mat A2 = (cv::Mat_<double>(3, 4) <<
		f, 0, w / 2, 0,
		0, f, h / 2, 0,
		0, 0, 1, 0);

	// Final and overall transformation matrix
	cv::Mat transfo = A2 * (T * (R * A1));

	// Apply matrix transformation
	cv::warpPerspective(source, destination, transfo, taille, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
	return destination;
}

void SetLightSource()
{
	float light_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
	float light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	float light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

	glEnable(GL_LIGHTING);                           //�}�O

	// �]�w�o���骺�������S��
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);      //���ҥ�(Ambient Light)
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);      //���g��(Diffuse Light)
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);      //�Ϯg��(Specular Light)

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);      //�����y��

	glEnable(GL_LIGHT0);                           //�}�ҹs���O
	glEnable(GL_DEPTH_TEST);
}

void SetMaterial()
{
	float material_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
	float material_diffuse[] = { 0.1, 0.1, 0.1, 1.0 };
	float material_specular[] = { 0.1, 0.1, 0.1, 1.0 };

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);
}

void Texture(void)
{
	glGenTextures(6, textures);
	glPrioritizeTextures(6, textures, priorities);
	SetTexObj("Debug\\bmpImage\\apple2.bmp", 0);
	SetTexObj("Debug\\bmpImage\\greenapple.bmp", 1);
	SetTexObj("Debug\\bmpImage\\apple2.bmp", 2);
	SetTexObj("Debug\\bmpImage\\apple2.bmp", 3);
	SetTexObj("Debug\\bmpImage\\apple2.bmp", 4);
	SetTexObj("Debug\\bmpImage\\apple2.bmp", 5);
	//glEnable(GL_TEXTURE_2D);

	ball = gluNewQuadric();
	gluQuadricTexture(ball, GL_TRUE);
}

void SetTexObj(char *name, int i)
{
	glBindTexture(GL_TEXTURE_2D, textures[i]);
	int width;
	int height;
	unsigned char *image;                     //��m���ɡA�w�g���OBMP�ϤF�A�O�ઽ����OpenGL�ϥΪ���ƤF
	BITMAPINFO bmpinfo;                        //�ΨӦs��HEADER��T

	image = LoadBitmapFile(name, &bmpinfo);
	width = bmpinfo.bmiHeader.biWidth;
	height = bmpinfo.bmiHeader.biHeight;

	//���豱��
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexImage2D(GL_TEXTURE_2D,0,3,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,image);

	//�ϥΦh����
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
}

unsigned char *LoadBitmapFile(char *fileName, BITMAPINFO *bitmapInfo)
{
	FILE            *fp;
	BITMAPFILEHEADER   bitmapFileHeader;   // Bitmap file header
	unsigned char       *bitmapImage;      // Bitmap image data
	unsigned int      lInfoSize;         // Size of information
	unsigned int      lBitSize;         // Size of bitmap

	unsigned char change;
	int pixel;
	int p = 0;

	fp = fopen(fileName, "rb");
	fread(&bitmapFileHeader, sizeof(BITMAPFILEHEADER), 1, fp);         //Ū�� bitmap header

	lInfoSize = bitmapFileHeader.bfOffBits - sizeof(BITMAPFILEHEADER);   //Info��size
	fread(bitmapInfo, lInfoSize, 1, fp);


	lBitSize = bitmapInfo->bmiHeader.biSizeImage;                  //�t�m�O����
	bitmapImage = new BYTE[lBitSize];
	fread(bitmapImage, 1, lBitSize, fp);                        //Ū���v����

	fclose(fp);

	//���ɶǦ^bitmapImage���ܡA�C��|�OBGR���ǡA�U���j��|�ﶶ�Ǭ�RGB
	pixel = (bitmapInfo->bmiHeader.biWidth)*(bitmapInfo->bmiHeader.biHeight);

	for (int i = 0; i<pixel; i++, p += 3)
	{
		//�洫bitmapImage[p]�MbitmapImage[p+2]����
		change = bitmapImage[p];
		bitmapImage[p] = bitmapImage[p + 2];
		bitmapImage[p + 2] = change;
	}

	return bitmapImage;
}

void loadObj(char *fname)
{
	FILE *fp;
	int read;
	GLfloat x, y, z;
	char ch;
	ballModel = glGenLists(1);
	fp = fopen(fname, "r");
	if (!fp)
	{
		printf("can't open file %s\n", fname);
		exit(1);
	}
	glPointSize(2.0);
	glNewList(ballModel, GL_COMPILE);
	{
		glPushMatrix();
		glBegin(GL_POINTS);
		while (!(feof(fp)))
		{
			read = fscanf(fp, "%c %f %f %f", &ch, &x, &y, &z);
			if (read == 4 && ch == 'v')
			{
				glVertex3f(x, y, z);
			}
		}
		glEnd();
	}
	glPopMatrix();
	glEndList();
	fclose(fp);
}

void drawBall()
{
	glPushMatrix();
	glTranslatef(ball_translatex, ball_translatey, -10.0);
	glRotatef(80.0, 0, 1, 0); //���ਤ�� , x ,y ,z
	gluSphere(ball, radius, 36, 72);// �b�|, �a����, ����� 
	glPopMatrix();
}
void drawBall2()
{
	glPushMatrix();
	glTranslatef(ball_translatex, ball_translatey, -10.0);
	glRotatef(180, 0, 0, 1); //���ਤ�� , x ,y ,z
	glRotatef(130, 0, 1, 0); //���ਤ�� , x ,y ,z
	glRotatef(40, 1, 0, 0); //���ਤ�� , x ,y ,z
	gluSphere(ball, radius, 36, 72);// �b�|, �a����, ����� 
	glPopMatrix();

}
