#include "chessboardcalibration.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


ChessboardCalibration::ChessboardCalibration( apicamera::Camera *camPtr, unsigned int _imgCountMax, unsigned int _cbWidth, unsigned int _cbHeight, float _squareSize) : Calibration(camPtr)
{
	//printf( "ici ChessboardCalibration::ChessboardCalibration()\n");

	imgCountMax = _imgCountMax;
	cbWidth = _cbWidth;
	cbHeight = _cbHeight;
	squareSize = _squareSize;

	int cornerCount = (cbHeight-1)*(cbWidth-1);		// inner corner count

	// allocate memory for 2D chessboard corners, for imgCountMax frames
	cb2DCorners = new CvPoint2D32f[imgCountMax*cornerCount];
	if( cb2DCorners == NULL )
	{
		printf("ERROR in ChessboardCalibration::ChessboardCalibration : memory allocation for 2D points failed.\n");
		exit(1);
	}
	
	// allocate memory for 3D chessboard corners, for imgCountMax frames
	cb3DCorners = new CvPoint3D32f[imgCountMax*cornerCount];
	if( cb3DCorners == NULL )
	{
		printf("ERROR in ChessboardCalibration::ChessboardCalibration : memory allocation for 3D points failed.\n");
		exit(1);
	}

	cb2DCornersSetIndex = 0;
}


ChessboardCalibration::~ChessboardCalibration()
{
	//printf( "ici ChessboardCalibration::~ChessboardCalibration()\n");

	delete[] cb2DCorners;
	delete[] cb3DCorners;
}


float ChessboardCalibration::findIntrinsicParameters(char *windowName)
{
	IplImage *currentImage;

	// accumulate 2D points sets
	resetCb2DCornersSetIndex(); 
	while( cb2DCornersSetIndex < imgCountMax )
	{
		// get frame
		currentImage = camera->get1Frame();

		// search 2D points
		add2DCornersSet(currentImage);
	
		// show image width 2D points
		if( windowName != NULL )
		{
			cvShowImage( windowName, currentImage);
			cvWaitKey(2);
		}
	}

	// calculate intrinsic parameters
	camera->intrinsicError = calcIntrinsicParameters();

	return  camera->intrinsicError;
}


int ChessboardCalibration::add2DCornersSet(IplImage *image)
{
	int cornerCount = (cbHeight-1)*(cbWidth-1);		// inner corner count

	if( getCb2DCornersSetCount() >= imgCountMax )
		// max points sets reached
		return -1; // failed, enough points

	if( extract2DPoints( image, &(cb2DCorners[cb2DCornersSetIndex*cornerCount])) )
	{
		incCb2DCornersSetIndex();
		imageSize = cvGetSize(image);
		return 1; // successfull
	}
	else
		return 0; // failed, no corners detected in image
}


float ChessboardCalibration::findExtrinsicParameters( float dx, float dy, char *windowName)
{
	IplImage *currentImage;

	// get frame
	currentImage = camera->get1Frame();

	// compute extrinsic parameters in image
	camera->extrinsicError = findExtrinsicParameters( dx, dy, currentImage);
		
	// show image width 2D points
	if( windowName != NULL )
		cvShowImage( windowName, currentImage);
	
	return  camera->extrinsicError;
}


float ChessboardCalibration::findExtrinsicParameters( float dx, float dy, IplImage *image)
{
	float error = 999999.0;

	// look for one set of 2D points
	resetCb2DCornersSetIndex(); 
	if( add2DCornersSet(image) == 1 )
	{
		// calculate chessboard points 3D position
		calc3DPoints( dx, dy);

		// calculate intrinsic parameters
		error = calcExtrinsicParameters();
	}
	
	return  error;
}


int ChessboardCalibration::extract2DPoints( IplImage *image, CvPoint2D32f* pcb2DCorners)
{
	int cornerCount = (cbHeight-1)*(cbWidth-1);
	int cornerCountFounded = 0;

	/*
	if( cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners) == 0 )
		// extraction failed
		return false;
	*/	

#if 0 // very slow
	// Brice way
	if( cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded, CV_CALIB_CB_ADAPTIVE_THRESH) == 0
		&& 	cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE) == 0
		&&  cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded,
CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS) == 0
		&&  cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded, CV_CALIB_CB_ADAPTIVE_THRESH) == 0
		&&  cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded,
CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) == 0 )
	{
		// extraction failed
		cvDrawChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, cornerCountFounded, 0);
		return false;
	}
#else
	// faster
	if( cvFindChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, &cornerCountFounded, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK) == 0 )
	{
		// extraction failed
		cvDrawChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, cornerCountFounded, 0);
		return false;
	}
#endif
	
	// refines the corner locations
	IplImage* grayImage = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1);
	if( image->nChannels == 3 )
		cvCvtColor( image, grayImage, CV_BGR2GRAY);
	else
		cvCopyImage( image, grayImage);
	
	cvFindCornerSubPix( grayImage, pcb2DCorners, cornerCount, cvSize( 5, 5), cvSize( 5, 5), cvTermCriteria( CV_TERMCRIT_ITER, 100, 0.1));
	
	cvReleaseImage(&grayImage);
	
	// draw corners
	cvDrawChessboardCorners( image, cvSize( cbWidth-1, cbHeight-1), pcb2DCorners, cornerCount, 1);
	
	return true;
}


void ChessboardCalibration::calc3DPoints( float dx, float dy)
{
	int cornerCount = (cbHeight-1)*(cbWidth-1);		// inner corner count
	int imgCount = getCb2DCornersSetCount();
	
	// calculate 3D positions
	int h = cbHeight-1;		// vertical inner corner count
	int w = cbWidth-1;		// horizontal inner corner count
	for( int iImg = 0; iImg < imgCount; iImg++)
		for( int i = 0; i < h; i++)
			for( int j = 0; j < w; j++)
				cb3DCorners[ iImg*h*w + i*w + j ] = cvPoint3D32f( j*squareSize + dx, i*squareSize + dy, 0 );	
}


float ChessboardCalibration::calcIntrinsicParameters(void)
{
   	int cornerCount = (cbHeight-1)*(cbWidth-1);		// inner corner count
	int imgCount = getCb2DCornersSetCount();
	CvSize imgSize = imageSize;
	
	// calculate chessboard points 3D position
	calc3DPoints( 0.0, 0.0);

	// inner corner count for each image used in calibration 
	int* pointCounts = new int[imgCount];
	for( int iImg = 0; iImg < imgCount; iImg++)
		pointCounts[iImg] = cornerCount;
	
	// calibrate
	float* R = new float[ imgCount * 3 * 3 ];
	float* T = new float[ imgCount * 3 ];
	cvCalibrateCamera( imgCount, pointCounts, imgSize, cb2DCorners, cb3DCorners, camera->intrinsicK, camera->intrinsicA, T, R, CV_CALIB_FIX_PRINCIPAL_POINT);
	
	// calculate calibration error
	CvPoint2D32f* ip = new CvPoint2D32f[cornerCount];
	CvPoint3D32f* op = new CvPoint3D32f[cornerCount];
	float err = 0, err3 = 0;
	for( int i = 0; i < imgCount; i ++) 
	{
		memcpy( camera->extrinsicR, R + i * 3 * 3, 3 * 3 * sizeof(float));
		memcpy( camera->extrinsicT, T + i * 3, 3 * sizeof(float));
		back2DProjection( cornerCount, cb3DCorners + i * cornerCount, ip, true);
		back3DProjection( cornerCount, op, cb2DCorners + i * cornerCount);
		err += error2D( cornerCount, cb2DCorners + i * cornerCount, ip);
		err3 += error3D( cornerCount, cb3DCorners + i * cornerCount, op);
	}

	delete [] op;
	delete [] ip;
	delete [] T;
	delete [] R;
	delete [] pointCounts;
	
	return  err / imgCount;
}


float ChessboardCalibration::calcExtrinsicParameters(void)
{
   	int cornerCount = (cbHeight-1)*(cbWidth-1);		// inner corner count
	CvPoint2D32f* ip = new CvPoint2D32f[cornerCount];
	CvPoint3D32f* op = new CvPoint3D32f[cornerCount];
	VEC3  Rv;
	CvMat _dist = cvMat( 1, 4, CV_32FC1, camera->intrinsicK);
	CvMat _A = cvMat( 3, 3, CV_32FC1, camera->intrinsicA);
	CvMat _r = cvMat( 1, 3, CV_32FC1, Rv);
	CvMat _t = cvMat( 1, 3, CV_32FC1, camera->extrinsicT);
	CvMat _ngrid = cvMat( 1, 1, CV_32SC1, &cornerCount);
	CvMat cr2 = cvMat( cornerCount, 1, CV_32FC2, cb2DCorners);
	CvMat cr3 = cvMat( cornerCount, 1, CV_32FC3, cb3DCorners);

	// calibrate
	cvFindExtrinsicCameraParams2( &cr3, &cr2, &_A, &_dist, &_r, &_t);
	CvMat _R = cvMat( 3, 3, CV_32FC1, camera->extrinsicR);
	cvRodrigues2( &_r, &_R, NULL);
	
	// calculate calibration error
	float err = 0, err3 = 0;
	back2DProjection( cornerCount, cb3DCorners, ip, true);
	back3DProjection( cornerCount, op, cb2DCorners);
	err = error2D( cornerCount, cb2DCorners, ip);
	err3 = error3D( cornerCount, cb3DCorners, op);

	delete [] op;
	delete [] ip;

	return err;
}


void ChessboardCalibration::back2DProjection( int point_count, const CvPoint3D32f* object_points, CvPoint2D32f* projected_points, bool use_dist) 
{
	int i;
	float pt[3];
        
	for( i = 0; i < point_count; i++) 
	{
		pt[0] = camera->extrinsicR[0]*object_points[i].x + camera->extrinsicR[1]*object_points[i].y + camera->extrinsicR[2]*object_points[i].z + camera->extrinsicT[0];
		pt[1] = camera->extrinsicR[3]*object_points[i].x + camera->extrinsicR[4]*object_points[i].y + camera->extrinsicR[5]*object_points[i].z + camera->extrinsicT[1];
		pt[2] = camera->extrinsicR[6]*object_points[i].x + camera->extrinsicR[7]*object_points[i].y + camera->extrinsicR[8]*object_points[i].z + camera->extrinsicT[2];

		pt[0] = pt[0] / pt[2];
		pt[1] = pt[1] / pt[2];
		pt[2] = 1.0f;

		if( use_dist ) 
			distortion( pt, camera->intrinsicK);

		projected_points[i].x = camera->intrinsicA[0]*pt[0] + camera->intrinsicA[1]*pt[1] + camera->intrinsicA[2];
		projected_points[i].y = camera->intrinsicA[3]*pt[0] + camera->intrinsicA[4]*pt[1] + camera->intrinsicA[5];
	}
}


void ChessboardCalibration::distortion( float* pf, const float* disto) 
{
	float u = pf [ 0 ];
	float v = pf [ 1 ];
	float r = u*u+v*v;
	
	pf[0] = u*(1.0f + disto[0]*r + disto[1]*r*r) + 2*u*v*disto[2] + disto[3]*(r+2*u*u);
	pf[1] = v*(1.0f + disto[0]*r + disto[1]*r*r) + 2*u*v*disto[3] + disto[2]*(r+2*v*v);
}       


void ChessboardCalibration::back3DProjection( int point_count, CvPoint3D32f* object_points, const CvPoint2D32f* projected_points) 
{
	int i;
	VEC4  pc, pg;
	VEC2  pi;
	MAT44  Rcg; 
	VEC4  Tcg;
	//MAT44  extR44;
	VEC4  extT4;
	
	mat33_to44( Rcg, camera->extrinsicR);
	vec4_init_v3dir( extT4, camera->extrinsicT);
	
	mat44_mul_vec( Tcg, Rcg, extT4);
	vec4_const_mul( Tcg, -1.0f, Tcg);
	for( i = 0; i < point_count; i++)
	{
		pi[0] = projected_points[i].x;
		pi[1] = projected_points[i].y;
		pixel_to_camera( pc, pi);
		mat44_mul_vec( pg, Rcg, pc);
		vec4_norm( pg, pg);
		float t = - Tcg[2] / pg[2];
		object_points[i].x = Tcg[0] + pg[0]*t;
		object_points[i].y = Tcg[1] + pg[1]*t;
		object_points[i].z = Tcg[2] + pg[2]*t;
	}
}


void ChessboardCalibration::undistortion( const MAT33 a, const VEC4 k, const float u, const float v, float& rx, float& ry)  
{
	float u0, v0, fx, fy, _fx, _fy, k1, k2, p1, p2;
	u0 = a[2]; v0 = a[5];
	fx = a[0]; fy = a[4];
	_fx = 1.f/fx; _fy = 1.f/fy;
	k1 = k[0]; k2 = k[1];
	p1 = k[2]; p2 = k[3];

	float y = (v - v0)*_fy;
	float x = (u - u0)*_fx;

	float y2 = y*y;
	float _2p1y = 2*p1*y;
	float _3p1y2 = 3*p1*y2;
	float p2y2 = p2*y2;

	float x2 = x*x;
	float r2 = x2 + y2;
	float d = 1 + (k1 + k2*r2)*r2;

	rx = (x*(d + _2p1y) + p2y2 + (3*p2)*x2);
	ry = (y*(d + (2*p2)*x) + _3p1y2 + p1*x2);
}


void ChessboardCalibration::pixel_to_camera( VEC4 c_c, const VEC2 c_i)  
{
	undistortion( camera->intrinsicA, camera->intrinsicK, c_i [ 0 ], c_i [ 1 ], c_c [ 0 ], c_c [ 1 ] );
	c_c [ 2 ] = 1.0f;
}


float ChessboardCalibration::error2D( int npts, CvPoint2D32f* pts2d1, CvPoint2D32f* pts2d2 ) {
	float err = 0.0f;
	int i;
	for ( i = 0 ; i < npts ; i ++ ) {
		float dx = pts2d1 [ i ].x - pts2d2 [ i ].x;
		float dy = pts2d1 [ i ].y - pts2d2 [ i ].y;
		err += sqrtf ( dx * dx + dy * dy );
	}
	return err / npts;
}


float ChessboardCalibration::error3D( int npts, CvPoint3D32f* pts2d1, CvPoint3D32f* pts2d2 ) {
	float err = 0.0f;
	int i;
	for ( i = 0 ; i < npts ; i ++ ) {
		float dx = pts2d1 [ i ].x - pts2d2 [ i ].x;
		float dy = pts2d1 [ i ].y - pts2d2 [ i ].y;
		float dz = pts2d1 [ i ].z - pts2d2 [ i ].z;
		err += sqrtf ( dx * dx + dy * dy + dz * dz );
	}
	return err / npts;
}




