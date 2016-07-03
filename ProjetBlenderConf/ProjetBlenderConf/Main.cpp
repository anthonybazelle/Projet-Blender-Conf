#pragma once



////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////PROTO 0////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

const int CV_AR_MARKER_SIZE = 160 ;				// Marker decoding size = 160 * 160 Pixels
const double CV_AR_DISP_SCALE_FIT = 0.0 ;		// Distort (& Fit) the Display Image
const double CV_AR_DISP_SCALE_DEF = 0.5 ;		// Scale the Display Image

bool cv_checkCorner(char* img_data,int img_width,int x,int y);		// Routine to check whether a particular pixel is an Edgel or not
void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B); 			// Routine to update Bounding Box corners
void cv_ARgetMarkerPoints(int points_total,CvPoint corners[10000],CvPoint START,CvPoint END, CvPoint& P,CvPoint& Q,CvPoint& R,CvPoint& S);// Routine to get 4-corners wrt (+)region partitioning
void cv_ARgetMarkerPoints2(int points_total,CvPoint corners[10000],CvPoint START,CvPoint END,CvPoint& L,CvPoint& M,CvPoint& N,CvPoint& O);// Routine to get 4-corners wrt (x)region partitioning
void cv_updateCorner(CvPoint quad,CvPoint box,double& dist, CvPoint& corner); 	// Distance algorithm for 4-corner validation
void cv_ARgetMarkerNum(int marker_id, int& marker_num);				// Routine to identify User specified number for Marker
void cv_ARgetMarkerID_16b(IplImage* img, int& marker_id);			// Routine to calculate Marker 16 bit ID
void cv_ARaugmentImage(IplImage* display,IplImage* img, CvPoint2D32f srcQuad[4], double scale = CV_AR_DISP_SCALE_DEF);	// Augment the display object on the Raw image
void cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img); 	// Routine to Mark and draw line on identified Marker Boundaries
void cv_lineEquation(CvPoint p1, CvPoint p2, double (&c)[3]);		// Equation of the line ax+by+c=0; a=c[0], b=c[1], c=c[2] for (x)region 4-corner detection
double cv_distanceFormula(double c[3],CvPoint p);					// Perpendicular distance of a point wrt a line ax+by+c=0; will be +ve or -ve depending upon position of point wrt line


int main()
{
    CvCapture *capture = 0;
    IplImage  *image = 0;
	IplImage *frame = 0;
	IplImage *disp,*neg_img,*cpy_img;
    int key = 0;
	int	fcount = 0;
	int option = 0;
 
    capture = cvCaptureFromCAM( 0 );
	if ( !capture ) 
        return -1;

	//Use a video with aspect ratio 4:3
	CvCapture* vid = cvCreateFileCapture("trailer.avi");
	if ( !vid )
       return -1;

	IplImage *pic = cvLoadImage("pic.jpg");
	cvFlip(pic,pic,1);
 
	int b_width  = 5;
	int b_height = 4;
	int b_squares = 20;
	CvSize b_size = cvSize( b_width, b_height );
	//The pattern actually has 6 x 5 squares, but has 5 x 4 = 20 'ENCLOSED' corners

	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
	CvPoint2D32f* corners = new CvPoint2D32f[ b_squares ];
	int corner_count;

	printf("Select an option to run the program\n\n");
	printf("1. Show an Image over the pattern.\n");
	printf("2. Play a Clip over the pattern.\n");
	printf("3. Mark the pattern.\n\n");
	scanf("%d",&option);
		
	//Quit on invalid entry
	if(!(option>=1 && option<=3))
	{
		printf("Invalid selection.");
		return -1;
	}

	cvNamedWindow("Video",CV_WINDOW_AUTOSIZE);

	while(key!='q') 
	{
		image = cvQueryFrame( capture );
		if( !image ) break;
		cvFlip(image,image,1);

		disp = cvCreateImage( cvGetSize(image), 8, 3 );
		cpy_img = cvCreateImage( cvGetSize(image), 8, 3 );
        neg_img = cvCreateImage( cvGetSize(image), 8, 3 );

		IplImage* gray = cvCreateImage( cvGetSize(image), image->depth, 1);
		int found = cvFindChessboardCorners(image, b_size, corners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		cvCvtColor(image, gray, CV_BGR2GRAY);
		
		//This function identifies the pattern from the gray image, saves the valid group of corners
		cvFindCornerSubPix(gray, corners, corner_count,  cvSize(11,11),cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
   
		if( corner_count == b_squares ) 
	    {
			if(option == 1)
			{
				CvPoint2D32f p[4];
				CvPoint2D32f q[4];

				IplImage* blank  = cvCreateImage( cvGetSize(pic), 8, 3);
				cvZero(blank);
				cvNot(blank,blank);
				
				//Set of source points to calculate Perspective matrix
				q[0].x= (float) pic->width * 0;
				q[0].y= (float) pic->height * 0;
				q[1].x= (float) pic->width;
				q[1].y= (float) pic->height * 0;

				q[2].x= (float) pic->width;
				q[2].y= (float) pic->height;
				q[3].x= (float) pic->width * 0;
				q[3].y= (float) pic->height;
		
				//Set of destination points to calculate Perspective matrix
				p[0].x= corners[0].x;
				p[0].y= corners[0].y;
				p[1].x= corners[4].x;
				p[1].y= corners[4].y;
				
				p[2].x= corners[19].x;
				p[2].y= corners[19].y;
				p[3].x= corners[15].x;
				p[3].y= corners[15].y;
				
				//Calculate Perspective matrix
				cvGetPerspectiveTransform(q,p,warp_matrix);

				//Boolean juggle to obtain 2D-Augmentation
				cvZero(neg_img);
				cvZero(cpy_img);

				cvWarpPerspective( pic, neg_img, warp_matrix);
				cvWarpPerspective( blank, cpy_img, warp_matrix);
				cvNot(cpy_img,cpy_img);

				cvAnd(cpy_img,image,cpy_img);
				cvOr(cpy_img,neg_img,image);

				cvShowImage( "Video", image); 
			}
			else if(option == 2)
			{
				CvPoint2D32f p[4];
				CvPoint2D32f q[4];

				frame = cvQueryFrame(vid);
				if (!frame)
				printf("error frame");

				IplImage* blank  = cvCreateImage( cvGetSize(frame), 8, 3);
				cvZero(blank);
				cvNot(blank,blank);

				q[0].x= (float) frame->width * 0;
				q[0].y= (float) frame->height * 0;
				q[1].x= (float) frame->width;
				q[1].y= (float) frame->height * 0;

				q[2].x= (float) frame->width;
				q[2].y= (float) frame->height;
				q[3].x= (float) frame->width * 0;
				q[3].y= (float) frame->height;
		
				p[0].x= corners[0].x;
				p[0].y= corners[0].y;
				p[1].x= corners[4].x;
				p[1].y= corners[4].y;
				
				p[2].x= corners[19].x;
				p[2].y= corners[19].y;
				p[3].x= corners[15].x;
				p[3].y= corners[15].y;
				
				cvGetPerspectiveTransform(q,p,warp_matrix);

				//Boolean juggle to obtain 2D-Augmentation
				cvZero(neg_img);
				cvZero(cpy_img);

				cvWarpPerspective( frame, neg_img, warp_matrix);
				cvWarpPerspective( blank, cpy_img, warp_matrix);
				cvNot(cpy_img,cpy_img);

				cvAnd(cpy_img,image,cpy_img);
				cvOr(cpy_img,neg_img,image);

				cvShowImage( "Video", image); 
			}
			else
			{
				CvPoint p[4];

				p[0].x=(int)corners[0].x;
				p[0].y=(int)corners[0].y;
				p[1].x=(int)corners[4].x;
				p[1].y=(int)corners[4].y;
				
				p[2].x=(int)corners[19].x;
				p[2].y=(int)corners[19].y;
				p[3].x=(int)corners[15].x;
				p[3].y=(int)corners[15].y;
				
				cvLine( image, p[0], p[1], CV_RGB(255,0,0),2);
				cvLine( image, p[1], p[2], CV_RGB(0,255,0),2);
				cvLine( image, p[2], p[3], CV_RGB(0,0,255),2);
				cvLine( image, p[3], p[0], CV_RGB(255,255,0),2);

				//or simply
				//cvDrawChessboardCorners(image, b_size, corners, corner_count, found);
				
				cvShowImage( "Video", image); 
			}
		}
		else
		{
			//Show gray image when pattern is not detected
			cvFlip(gray,gray);
			cvShowImage( "Video", gray );

		}
		key = cvWaitKey(1);
		
	}

    cvDestroyWindow( "Video" );
    cvReleaseCapture( &vid );
	cvReleaseMat(&warp_matrix);
	cvReleaseCapture( &capture );

    return 0;
}
// Start of Main Loop
//------------------------------------------------------------------------------------------------------------------------
/*
int main ( int argc, char **argv )
{
	CvCapture* capture = 0;
	IplImage* img = 0;

	// List of Images to be augmented on the Marker

	IplImage* display_img1  = cvLoadImage("image_ar.jpg");			
		if ( !display_img1 )
		return -1;

	capture = cvCaptureFromCAM( 0 );
		if ( !capture )           	// Check for Camera capture
		return -1;

	cvNamedWindow("Camera",CV_WINDOW_AUTOSIZE);

//	cvNamedWindow("Test",CV_WINDOW_AUTOSIZE);	// Test window to push any visuals during debugging

	IplImage* gray = 0;
	IplImage* thres = 0;
	IplImage* prcs_flg = 0;		// Process flag to flag whether the current pixel is already processed as part blob detection


	int q,i;					// Intermidiate variables
	int h,w;					// Variables to store Image Height and Width

	int ihist[256];				// Array to store Histogram values
	float hist_val[256];		// Array to store Normalised Histogram values

	int blob_count;
	int n;						// Number of pixels in a blob
	int pos ;					// Position or pixel value of the image

	int rectw,recth;			// Width and Height of the Bounding Box
	double aspect_ratio;		// Aspect Ratio of the Bounding Box

    int min_blob_sze = 50;              	// Minimum Blob size limit 
 // int max_blob_sze = 150000;           	// Maximum Blob size limit

	CvPoint P,Q,R,S;			// Corners of the Marker

	// Note: All Markers are tranposed to 160 * 160 pixels Image for decoding
	IplImage* marker_transposed_img = cvCreateImage( cvSize(CV_AR_MARKER_SIZE,CV_AR_MARKER_SIZE), 8, 1 );

	CvPoint2D32f srcQuad[4], dstQuad[4];	// Warp matrix Parameters: Source, Destination

        dstQuad[0].x = 0;			// Positions of Marker image (to where it has to be transposed)
        dstQuad[0].y = 0;
        dstQuad[1].x = CV_AR_MARKER_SIZE;
        dstQuad[1].y = 0;
        dstQuad[2].x = 0;
        dstQuad[2].y = CV_AR_MARKER_SIZE;
        dstQuad[3].x = CV_AR_MARKER_SIZE;
        dstQuad[3].y = CV_AR_MARKER_SIZE;

	bool init = false;			// Flag to identify initialization of Image objects



	//Step	: Capture a frame from Camera for creating and initializing manipulation variables
	//Info	: Inbuit functions from OpenCV
	//Note	: 

    	if(init == false)
	{
        	img = cvQueryFrame( capture );	// Query for the frame
        		if( !img )		// Exit if camera frame is not obtained
			return -1;

		// Creation of Intermediate 'Image' Objects required later
		gray = cvCreateImage( cvGetSize(img), 8, 1 );		// To hold Grayscale Image
		thres = cvCreateImage( cvGetSize(img), 8, 1 );		// To hold OTSU thresholded Image
		prcs_flg = cvCreateImage( cvGetSize(img), 8, 1 );	// To hold Map of 'per Pixel' Flag to keep track while identifing Blobs

		
		init = true;
	}

	int clr_flg[img->width];	// Array representing elements of entire current row to assign Blob number
	int clrprev_flg[img->width];	// Array representing elements of entire previous row to assign Blob number

	h = img->height;		// Height and width of the Image
	w = img->width;

	bool corner_flag = false;      				// Flag to check whether the current pixel is a Edgel or not
	CvPoint corners[10000];         			// Array to store all the Edgels.If the size of the array is small then there may be abrupt termination of the program

	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);         // Warp matrix to store perspective data


	double t;			// variable to calculate timing

	int marker_id;  
 	int marker_num;  
	bool valid_marker_found;
	
	//For Recording Output
	//double fps = cvGetCaptureProperty(capture,  CV_CAP_PROP_FPS );
	//CvVideoWriter *writer = cvCreateVideoWriter( "out.avi",  CV_FOURCC('M', 'J', 'P', 'G'), 24, cvGetSize(img) );

	int key = 0;
	while(key != 'q')		// While loop to query for Camera frame
	{
	    t = cvGetTickCount();

		//Step	: Capture Image from Camera
		//Info	: Inbuit function from OpenCV
		//Note	: 

		img = cvQueryFrame( capture );		// Query for the frame

		//Step	: Convert Image captured from Camera to GrayScale
		//Info	: Inbuit function from OpenCV
		//Note	: Image from Camera and Grayscale are held using seperate "IplImage" objects

		cvCvtColor(img,gray,CV_RGB2GRAY);	// Convert RGB image to Gray


		//Step	: Threshold the image using optimum Threshold value obtained from OTSU method
		//Info	: 
		//Note	: 

		memset(ihist, 0, 256);

		for(int j = 0; j < gray->height; ++j)	// Use Histogram values from Gray image
		{
			uchar* hist = (uchar*) (gray->imageData + j * gray->widthStep);
			for(int i = 0; i < gray->width; i++ )
			{
				pos = hist[i];		// Check the pixel value
				ihist[pos] += 1;	// Use the pixel value as the position/"Weight"
			}
		}

		//Parameters required to calculate threshold using OTSU Method
		float prbn = 0.0;                   // First order cumulative
		float meanitr = 0.0;                // Second order cumulative
		float meanglb = 0.0;                // Global mean level
		int OPT_THRESH_VAL = 0;             // Optimum threshold value
		float param1,param2;                // Parameters required to work out OTSU threshold algorithm
		double param3 = 0.0;

		//Normalise histogram values and calculate global mean level
		for(int i = 0; i < 256; ++i)
		{
			hist_val[i] = ihist[i] / (float)(w * h);
			meanglb += ((float)i * hist_val[i]);
		}

	    	// Implementation of OTSU algorithm
		for (int i = 0; i < 255; i++)
		{
			prbn += (float)hist_val[i];
			meanitr += ((float)i * hist_val[i]);

			param1 = (float)((meanglb * prbn) - meanitr);
			param2 = (float)(param1 * param1) /(float) ( prbn * (1.0f - prbn) );

			if (param2 > param3)
			{
			    param3 = param2;
			    OPT_THRESH_VAL = i; 				// Update the "Weight/Value" as Optimum Threshold value
			}
		}

		cvThreshold(gray,thres,OPT_THRESH_VAL,255,CV_THRESH_BINARY);	//Threshold the Image using the value obtained from OTSU method


		//Step	: Identify Blobs in the OTSU Thresholded Image
		//Info	: Custom Algorithm to Identify blobs
		//Note	: This is a complicated method. Better refer the presentation, documentation or the Demo

		blob_count = 0;			// Current Blob number used to represent the Blob
		CvPoint cornerA,cornerB; 	// Two Corners to represent Bounding Box

		memset(clr_flg, 0, w);		// Reset all the array elements ; Flag for tracking progress
		memset(clrprev_flg, 0, w);

		cvZero(prcs_flg);		// Reset all Process flags


        for( int y = 0; y < thres->height; ++y)	//Start full scan of the image by incrementing y
        {
            uchar* prsnt = (uchar*) (thres->imageData + y * thres->widthStep);
            uchar* pntr_flg = (uchar*) (prcs_flg->imageData + y * prcs_flg->widthStep);  // pointer to access the present value of pixel in Process flag
			uchar* scn_prsnt;      // pointer to access the present value of pixel related to a particular blob
			uchar* scn_next;       // pointer to access the next value of pixel related to a particular blob

            for(int x = 0; x < thres->width; ++x )	//Start full scan of the image by incrementing x
            {
                int c = 0;					// Number of edgels in a particular blob
                marker_id = 0;					// Identification number of the particular pattern
                if((prsnt[x] == 0) && (pntr_flg [x] == 0))	// If current pixel is black and has not been scanned before - continue
                {
			blob_count +=1;                          // Increment at the start of processing new blob
			clr_flg [x] = blob_count;                // Update blob number
			pntr_flg [x] = 255;                      // Mark the process flag

			n = 1;                                   // Update pixel count of this particular blob / this iteration

			cornerA.x = x;                           // Update Bounding Box Location for this particular blob / this iteration
			cornerA.y = y;
			cornerB.x = x;
			cornerB.y = y;

			int lx,ly;						// Temp location to store the initial position of the blob
			int belowx = 0;

			bool checkbelow = true;			// Scan the below row to check the continuity of the blob

                    ly=y;

                    bool below_init = 1;			// Flags to facilitate the scanning of the entire blob once
                    bool start = 1;

                        while(ly < h)				// Start the scanning of the blob
                        {
                            if(checkbelow == true)			// If there is continuity of the blob in the next row & checkbelow is set; continue to scan next row
                            {
                                if(below_init == 1) 		// Make a copy of Scanner pixel position once / initially
                                {
                                    belowx=x;
                                    below_init = 0;
                                }

                                checkbelow = false;		// Clear flag before next flag

                                scn_prsnt = (uchar*) (thres->imageData + ly * thres->widthStep);
                                scn_next = (uchar*) (thres->imageData + (ly+1) * thres->widthStep);

                                pntr_flg = (uchar*) (prcs_flg->imageData + ly * prcs_flg->widthStep);

                                bool onceb = 1;			// Flag to set and check blbo continuity for next row

                                // Loop to move Scanner pixel to the extreme left pixel of the blob
                                while((scn_prsnt[belowx-1] == 0) && ((belowx-1) > 0) && (pntr_flg[belowx-1]== 0))
                                {
                                    cv_adjustBox(belowx,ly,cornerA,cornerB);    // Update Bounding Box corners
                                    pntr_flg [belowx] = 255;

                                    clr_flg [belowx] = blob_count;

                                    corner_flag = cv_checkCorner(thres->imageData,thres->widthStep,belowx,ly);
                                    if(corner_flag == true)		// Check for the Edgel and update Edgel storage
                                    {
					if (c < 10000)			// Make sure the allocated array size does not exceed
		                        {
                                        corners[c].x=belowx;
                                        corners[c].y=ly;
                                        c++;
					}
                                        corner_flag = false;
                                    }
                                    n = n+1;
                                    belowx--;
                                }
                                //Scanning of a particular row of the blob
                                for(lx = belowx; lx < thres->width; ++lx )
                                {
                                    if(start == 1)                 	// Initial/first row scan
                                    {
                                        cv_adjustBox(lx,ly,cornerA,cornerB);
                                        pntr_flg [lx] = 255;

                                        clr_flg [lx] = blob_count;

                                        corner_flag = cv_checkCorner(thres->imageData,thres->widthStep,lx,ly);
                                        if(corner_flag == true)
                                        {
						if (c < 10000)					// Make sure the allocated array size does not exceed
		                                {
							corners[c].x = lx;
							corners[c].y = ly;
		                                    c++;
						}
                                            corner_flag = false;
                                        }

                                        start = 0;
                                        if((onceb == 1) && (scn_next[lx] == 0))                 // Check for the continuity
                                        {
                                            belowx = lx;
                                            checkbelow = true;
                                            onceb = 0;
                                        }
                                    }
                                    else if((scn_prsnt[lx] == 0) && (pntr_flg[lx] == 0))        // Present pixel is black and has not been processed
                                    {
                                        if((clr_flg[lx-1] == blob_count) || (clr_flg[lx+1] == blob_count))        //Check for the continuity with previous scanned data
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            corner_flag = cv_checkCorner(thres->imageData,thres->widthStep,lx,ly);
                                            if(corner_flag == true)
                                            {
						if (c < 10000)					// Make sure the allocated array size does not exceed
		                                {
                                                corners[c].x=lx;
                                                corners[c].y=ly;
                                                c++;
						}
                                                corner_flag = false;
                                            }
                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }
                                        }
                                        else if((scn_prsnt[lx] == 0) && (clr_flg[lx-2] == blob_count))	// Check for the continuity with previous scanned data
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            corner_flag = cv_checkCorner(thres->imageData,thres->widthStep,lx,ly);
                                            if(corner_flag == true)
                                            {
						if (c < 10000)					// Make sure the allocated array size does not exceed
		                                {
                                                corners[c].x=lx;
                                                corners[c].y=ly;
                                                c++;
						}
                                                corner_flag = false;
                                            }
                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }
                                        }
                                        // Check for the continuity with previous scanned data
                                        else if((scn_prsnt[lx] == 0) && ((clrprev_flg[lx-1] == blob_count) || (clrprev_flg[lx] == blob_count) || (clrprev_flg[lx+1] == blob_count)))
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            corner_flag = cv_checkCorner(thres->imageData,thres->widthStep,lx,ly);
                                            if(corner_flag == true)
                                            {
						if (c < 10000)					// Make sure the allocated array size does not exceed
		                                {
                                                corners[c].x=lx;
                                                corners[c].y=ly;
                                                c++;
						}
                                                corner_flag = false;
                                            }
                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }

                                        }
                                        else
                                        {
                                            continue;
                                        }

                                    }
                                    else
                                    {
                                        clr_flg[lx] = 0;	// Current pixel is not a part of any blob
                                    }
                                }				// End of scanning of a particular row of the blob
                            }
                            else				// If there is no continuity of the blob in the next row break from blob scan loop
                            {
                                break;
                            }

                            for(int q = 0; q < thres->width; ++q)	// Blob numbers of current row becomes Blob number of previous row for the next iteration of "row scan" for this particular blob
                            {
                                clrprev_flg[q]= clr_flg[q];
                            }
                            ly++;
                        }
                        // End of the Blob scanning routine 


			// At this point after scanning image data, A blob (or 'connected component') is obtained. We use this Blob for further analysis to confirm it is a Marker.

			
			// Get the Rectangular extent of the blob. This is used to estimate the span of the blob
			// If it too small, say only few pixels, it is too good to be true that it is a Marker. Thus reducing erroneous decoding
			rectw = abs(cornerA.x - cornerB.x);
			recth = abs(cornerA.y - cornerB.y);
			aspect_ratio = (double)rectw / (double)recth;

                        if((n > min_blob_sze))// && (n < max_blob_sze))		// Reduces chances of decoding erroneous 'Blobs' as markers
                        {
                            if((aspect_ratio > 0.33) && (aspect_ratio < 3.0))	// Increases chances of identified 'Blobs' to be close to Square 
                            {

				// Step	: Identify 4 corners of the blob assuming it be a potential Marker
				// Info	: Custom Algorithm to detect Corners using Pixel data || similar to FAST algorithm
				// Note	: 

				cv_ARgetMarkerPoints(c,corners,cornerA,cornerB,P,Q,R,S);      // 4-corners of the pattern obtained usig (+)region calculations

				// CvPoint to CvPoint2D32f conversion for Warp Matrix calculation

                                srcQuad[0].x = P.x;				// Positions of the Marker in Image | "Deformed" Marker
                                srcQuad[0].y = P.y;				
                                srcQuad[1].x = Q.x;
                                srcQuad[1].y = Q.y;
                                srcQuad[2].x = S.x;
                                srcQuad[2].y = S.y;
                                srcQuad[3].x = R.x;
                                srcQuad[3].y = R.y;


                               	// Note: dstQuad[4];				// Positions to where Marker has to be transposed to | "Aligned" Marker

				// Note: All Markers are tranposed to 160 * 160 pixels Image for decoding

				cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);		// Warp Matrix Calculations
				cvWarpPerspective(thres, marker_transposed_img, warp_matrix);	// SMART! Clip and Transform the deformed Marker simultaneously using a Mask (Marker catcher) and Warp Matrix 


				// Step	: Decode 16bit Marker to Identify marker uniquely and Get associated Marker Number
				// Info	: 
				// Note	: The Marker ID is valid in any 4 Direction of looking

                                cv_ARgetMarkerID_16b(marker_transposed_img, marker_id);	// Get Marker ID
				cv_ARgetMarkerNum(marker_id, marker_num);		// Get Marker Number Corrosponding to ID


				if (marker_num == 1)	
				{
					valid_marker_found = true;
				}
				else
				{
					// If 4-Corners are not obtained from (+) region partitioning ; try to calculate corners from (x) region partitioning
					cv_ARgetMarkerPoints2(c,corners,cornerA,cornerB,P,Q,R,S);

					srcQuad[0].x = P.x;			// Positions of the Marker in Image | "Deformed" Marker
		                        srcQuad[0].y = P.y;
		                        srcQuad[1].x = Q.x;
		                        srcQuad[1].y = Q.y;
		                        srcQuad[2].x = S.x;
		                        srcQuad[2].y = S.y;
		                        srcQuad[3].x = R.x;
		                        srcQuad[3].y = R.y;

					cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);		// Warp Matrix Calculations
					cvWarpPerspective(thres, marker_transposed_img, warp_matrix);	

		                        cv_ARgetMarkerID_16b(marker_transposed_img,marker_id);	// Get Marker ID
					cv_ARgetMarkerNum(marker_id, marker_num);		// Get Marker Number Corrosponding to I

				}
				
				if (marker_num == 1)				// Now check if still marker is valid
				{
					valid_marker_found = true;
				}

				if(valid_marker_found == true)			// Show Display image corrosponding to the Marker Number
                                {
				
					
					// Step	: Augment the "Display object" in position of the marker over Camera Image using the Warp Matrix
					// Info	: 
					// Note	: Marker number used to make it easlier to change 'Display' image accordingly, 
					// Also Note the jugglery to augment due to OpenCV's limiation passing two images of DIFFERENT sizes  
					// while using "cvWarpPerspective".  
					
					if(marker_num == 1)
					{
						cv_ARaugmentImage(display_img1, img, srcQuad, CV_AR_DISP_SCALE_FIT); // Send the Image to display, Camera Image and Position of the Marker		
						cv_ARoutlineMarker(cornerA,cornerB,P,Q,R,S,img);
						// cvShowImage( "Test", marker_transposed_img);
					}
				}
				valid_marker_found = false;

				// If a valid marker was detected, then a Image will be augmented on that blob and process will continue to analysis of next blob
                                
                            }	
                            else	// Discard the blob data
                            {                      
                                blob_count = blob_count -1;	
                            }
                        }
                        else  		// Discard the blob data               
                        {
                            blob_count = blob_count -1;		
                        }
                }
                else     // If current pixel is not black do nothing
                {
                    continue;
                }
	}	// End full scan of the image by incrementing x
        }	// End full scan of the image by incrementing y
	

		t = cvGetTickCount() - t;
		//printf("Calc. = %.4gms : FPS = %.4g\n",t/((double)cvGetTickFrequency()*1000.), ((double)cvGetTickFrequency()*1000.*1000.)/t);

		cvShowImage("Camera",img);

 		//cvWriteFrame( writer, img );		// Save frame to output

		key = cvWaitKey(1);			// OPENCV: wait for 1ms before accessing next frame

	}						// End of 'while' loop

	cvDestroyWindow( "Camera" );			// Release various parameters

	cvReleaseImage(&img);
	cvReleaseImage(&gray);
	cvReleaseImage(&thres);
	cvReleaseImage(&prcs_flg);
	cvReleaseImage(&marker_transposed_img);

	cvReleaseImage(&display_img1);

	cvReleaseMat(&warp_matrix);

	//cvReleaseVideoWriter( &writer );

    	return 0;
}
// End of Main Loop
//------------------------------------------------------------------------------------------------------------------------


// Routines used in Main loops

// Routine to update Bounding Box corners with farthest corners in that Box
void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B)
{
    if(x < A.x)
        A.x = x;

    if(y < A.y)
        A.y = y;

    if(x > B.x)
        B.x = x;

    if(y > B.y)
        B.y = y;
}

// Routine to check whether a particular pixel is an Edgel or not
bool cv_checkCorner(char* img_data,int img_width,int x,int y)
{
	const int wind_sz = 5;
	int wind_bnd = (wind_sz - 1) / 2;
	int sum = 0;
	bool result = false;
	uchar* ptr[wind_sz];
	int index =0;

	for(int k = (0-wind_bnd); k <= wind_bnd; ++k)
	{
		 ptr[index] = (uchar*)(img_data + (y + k) *  img_width);
		 index = index + 1 ;
	}

	for(int i = 0; i <= (wind_sz-1); ++i)
	{
		if((i == 0) || (i==(wind_sz-1)))
		{
		    for (int j = (0-wind_bnd); j <= wind_bnd; ++j)
		    {
			if(ptr[i][x+j] == 0)
			   sum += 1;
			else
			   continue;
		    }
		}
		else
		{
		    if(ptr[i][x-wind_bnd] == 0)
			sum += 1;
		    else
			continue;

		    if(ptr[i][x+wind_bnd] == 0)
			sum += 1;
		    else
			continue;
		}
	}

    if((sum >= 4) && (sum <= 12))
    {
        result = true;
    }
    return result;
}

// Distance algorithm for 4-corner validation
void cv_updateCorner(CvPoint quad,CvPoint box,double& dist, CvPoint& corner)
{
    double temp_dist;
    temp_dist = sqrt(((box.x - quad.x) * (box.x - quad.x)) + ((box.y - quad.y) * (box.y - quad.y)));

    if(temp_dist > dist)
    {
        dist = temp_dist;
        corner = quad;
    }
}

// Routine to calculate 4 Corners of the Marker in Image Space using (+) Region partitioning
void cv_ARgetMarkerPoints(int points_total,CvPoint corners[10000],CvPoint START,CvPoint END, CvPoint& P,CvPoint& Q,CvPoint& R,CvPoint& S)
{
    CvPoint A = START;
    CvPoint B;
    B.x = END.x;
    B.y = START.y;

    CvPoint C = END;
    CvPoint D;
    D.x = START.x;
    D.y = END.y;

    int halfx = (A.x + B.x) / 2;
    int halfy = (A.y + D.y) / 2;


    double dmax[4];
    dmax[0]=0.0;
    dmax[1]=0.0;
    dmax[2]=0.0;
    dmax[3]=0.0;

    for(int i = 0; i < points_total; ++i)
    {
        if((corners[i].x < halfx) && (corners[i].y <= halfy))
        {
            cv_updateCorner(corners[i],C,dmax[2],P);
        }
        else if((corners[i].x >= halfx) && (corners[i].y < halfy))
        {
            cv_updateCorner(corners[i],D,dmax[3],Q);
        }
        else if((corners[i].x > halfx) && (corners[i].y >= halfy))
        {
            cv_updateCorner(corners[i],A,dmax[0],R);
        }
        else if((corners[i].x <= halfx) && (corners[i].y > halfy))
        {
            cv_updateCorner(corners[i],B,dmax[1],S);
        }
    }

}

// Routine to calculate 4 Corners of the Marker in Image Space using (x) Region partitioning
void cv_ARgetMarkerPoints2(int points_total,CvPoint corners[10000],CvPoint START,CvPoint END,CvPoint& L,CvPoint& M,CvPoint& N,CvPoint& O)
{
    CvPoint A = START;
    CvPoint B;
    B.x = END.x;
    B.y = START.y;
    CvPoint C = END;
    CvPoint D;
    D.x = START.x;
    D.y = END.y;

    CvPoint W,X,Y,Z;

    double line1[3],line2[3];
    double pd1 = 0.0;
    double pd2 = 0.0;

    W.x = (START.x + END.x) / 2;
    W.y = START.y;

    X.x = END.x;
    X.y = (START.y + END.y) / 2;

    Y.x = (START.x + END.x) / 2;
    Y.y = END.y;

    Z.x = START.x;
    Z.y = (START.y + END.y) / 2;

    cv_lineEquation( C, A, line1);
    cv_lineEquation( B, D, line2);

    double rdmax[4];
    rdmax[0] = 0.0;
    rdmax[1] = 0.0;
    rdmax[2] = 0.0;
    rdmax[3] = 0.0;

    for(int i = 0; i < points_total; ++i)
    {
        pd1 = cv_distanceFormula(line1,corners[i]);
        pd2 = cv_distanceFormula(line2,corners[i]);

        if((pd1 >= 0.0) && (pd2 > 0.0))
        {
            cv_updateCorner(corners[i],W,rdmax[2],L);
        }
        else if((pd1 > 0.0) && (pd2 <= 0.0))
        {
            cv_updateCorner(corners[i],X,rdmax[3],M);
        }
        else if((pd1 <= 0.0) && (pd2 < 0.0))
        {
            cv_updateCorner(corners[i],Y,rdmax[0],N);
        }
        else if((pd1 < 0.0) && (pd2 >= 0.0))
        {
            cv_updateCorner(corners[i],Z,rdmax[1],O);
        }
        else
            continue;
    }
}


void cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img)
{
	cvRectangle(raw_img,Top,Bottom,CV_RGB(255,0,0),1);	// Draw untransfromed/flat rectangle on Bounding Box with Marker

	cvLine(raw_img, A, B, CV_RGB(255,0,0),2);		// Draw rectangle by joining 4 corners of the Marker via CVLINE
	cvLine(raw_img, B, C, CV_RGB(0,255,0),2);
	cvLine(raw_img, C, D, CV_RGB(0,0,255),2);
	cvLine(raw_img, D, A, CV_RGB(255,255,0),2);

	cvCircle(raw_img,A,4,CV_RGB(128,255,128),1,8);		// Mark 4 corners of the Marker in Green		
	cvCircle(raw_img,B,4,CV_RGB(128,255,128),1,8);
	cvCircle(raw_img,C,4,CV_RGB(128,255,128),1,8);
	cvCircle(raw_img,D,4,CV_RGB(128,255,128),1,8);
}


// Routine to calculate Binary Marker Idendifier 
void cv_ARgetMarkerID_16b(IplImage* img, int& marker_id)
{
	// Black is 1/True and White is 0/False for following Binary calculation

	int i=0;
	int value = 0;
	for(int y=50;y<120;y=y+20)
	{
		uchar* pointer_scanline =(uchar*) (img->imageData + (y-1)*img->width);

		for(int x = 50; x<120; x=x+20)
		{
			if(pointer_scanline[x-1]==0)
			{
				value += (int) pow(2,i); 		// Covert to decimal by totaling 2 raised to power of 'Bitmap position'
			}
			i++;
		}
	}
	marker_id = value;
}

void cv_ARgetMarkerNum(int marker_id, int& marker_num)
{
	marker_num = 0;
	if(marker_id == 63729 || marker_id == 47790 || marker_id == 36639 || marker_id == 30045)
	{
		marker_num = 1;
	
	}
}

void cv_ARaugmentImage(IplImage* display,IplImage* img, CvPoint2D32f srcQuad[4], double scale)
{
	
	IplImage* cpy_img = cvCreateImage( cvGetSize(img), 8, 3 );	// To hold Camera Image Mask 
	IplImage* neg_img = cvCreateImage( cvGetSize(img), 8, 3 );	// To hold Marker Image Mask
	IplImage* blank;						// To assist Marker Pass
	IplImage temp;						

	blank = cvCreateImage( cvGetSize(display), 8, 3 );
	cvZero(blank);
	cvNot(blank,blank);

	CvPoint2D32f dispQuad[4];
	CvMat* disp_warp_matrix = cvCreateMat(3,3,CV_32FC1);    // Warp matrix to store perspective data required for display

	if (scale == CV_AR_DISP_SCALE_FIT)
	{
		dispQuad[0].x = 0;				// Positions of Display image (not yet transposed)
		dispQuad[0].y = 0;

		dispQuad[1].x = display->width;
		dispQuad[1].y = 0;

		dispQuad[2].x = 0;
		dispQuad[2].y = display->height;

		dispQuad[3].x = display->width;
		dispQuad[3].y = display->height;	
	}
	else
	{	
		dispQuad[0].x = (display->width/2) - (CV_AR_MARKER_SIZE/scale);			// Positions of Display image (not yet transposed)
		dispQuad[0].y = (display->height/2)- (CV_AR_MARKER_SIZE/scale);

		dispQuad[1].x = (display->width/2) + (CV_AR_MARKER_SIZE/scale);
		dispQuad[1].y = (display->height/2)- (CV_AR_MARKER_SIZE/scale);

		dispQuad[2].x = (display->width/2) - (CV_AR_MARKER_SIZE/scale);
		dispQuad[2].y = (display->height/2)+ (CV_AR_MARKER_SIZE/scale);

		dispQuad[3].x = (display->width/2) + (CV_AR_MARKER_SIZE/scale);
		dispQuad[3].y = (display->height/2)+ (CV_AR_MARKER_SIZE/scale);
	}

	cvGetPerspectiveTransform(dispQuad,srcQuad,disp_warp_matrix);	// Caclculate the Warp Matrix to which Display Image has to be transformed

	// Note the jugglery to augment due to OpenCV's limiation passing two images [- Marker Img and Raw Img] of DIFFERENT sizes 
	// while using "cvWarpPerspective".  

	cvZero(neg_img);
	cvZero(cpy_img);
	cvWarpPerspective( display, neg_img, disp_warp_matrix);
	cvWarpPerspective( blank, cpy_img, disp_warp_matrix);
	cvNot(cpy_img,cpy_img);
	cvAnd(cpy_img,img,cpy_img);
	cvOr(cpy_img,neg_img,img);

	// Release images
	cvReleaseImage(&cpy_img);
	cvReleaseImage(&neg_img);
	cvReleaseImage(&blank);

	cvReleaseMat(&disp_warp_matrix);

}


// Equation of the line ax+by+c=0; a=c[0], b=c[1], c=c[2] for (x)region 4-corner detection
void cv_lineEquation(CvPoint p1, CvPoint p2, double (&c)[3])
{
    c[0] = -((double)(p2.y - p1.y) / (double)(p2.x - p1.x));
    c[1] = (double)1.0;
    c[2] = (((double)(p2.y - p1.y) / (double)(p2.x - p1.x)) * (double)p1.x) - (double)p1.y;

    return;
}

// Perpendicular distance of a point wrt a line ax+by+c=0; will be +ve or -ve depending upon position of point wrt linen
double cv_distanceFormula(double c[],CvPoint p)
{
    double pdist = 0.0;
    pdist = ((double)(c[0] * p.x) + (double)(c[1] * p.y) + (c[2])) / (sqrt((double)(c[0] * c[0]) + (double)(c[1] * c[1])));
    return pdist;
}
*/

// EOF
//______________________________________________________________________________________

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// PROTOTYPE 1 /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

const int CV_QR_NORTH = 0;
const int CV_QR_EAST = 1;
const int CV_QR_SOUTH = 2;
const int CV_QR_WEST = 3;

float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(vector<vector<Point> > contours, int c_id,float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1,Point2f v2);

// Start of Main Loop
//------------------------------------------------------------------------------------------------------------------------
int main ( int argc, char **argv )
{

	VideoCapture capture(0);

	//Mat image = imread(argv[1]);
	Mat image;

	if(!capture.isOpened()) { cerr << " ERR: Unable find input Video source." << endl;
		return -1;
	}

	//Step	: Capture a frame from Image Input for creating and initializing manipulation variables
	//Info	: Inbuilt functions from OpenCV
	//Note	: 
	
 	capture >> image;
	if(image.empty()){ cerr << "ERR: Unable to query image from capture device.\n" << endl;
		return -1;
	}
	

	// Creation of Intermediate 'Image' Objects required later
	Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image
	Mat edges(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image
	Mat traces(image.size(), CV_8UC3);								// For Debug Visuals
	Mat qr,qr_raw,qr_gray,qr_thres;
	    
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	int mark,A,B,top,right,bottom,median1,median2,outlier;
	int C = 0;																	// Problème variable C, jamais initialisée sans cette ligne, Hors limite tableau de vecteurs
	float AB,BC,CA, dist,slope, areat,arear,areab, large, padding;
	
	int align,orientation;

	int DBG=1;						// Debug Flag

	int key = 0;
	while(key != 'q')				// While loop to query for Image Input frame
	{

		traces = Scalar(0,0,0);
		qr_raw = Mat::zeros(100, 100, CV_8UC3 );
	   	qr = Mat::zeros(100, 100, CV_8UC3 );
		qr_gray = Mat::zeros(100, 100, CV_8UC1);
	   	qr_thres = Mat::zeros(100, 100, CV_8UC1);		
		
		capture >> image;						// Capture Image from Image Input

		cvtColor(image,gray,CV_RGB2GRAY);		// Convert Image captured from Image Input to GrayScale	
		Canny(gray, edges, 100 , 200, 3);		// Apply Canny edge detection on the gray image


		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

		mark = 0;								// Reset all detected marker count for this frame

		// Get Moments for all Contours and the mass centers
		vector<Moments> mu(contours.size());
  		vector<Point2f> mc(contours.size());

		for( int i = 0; i < contours.size(); i++ )
		{	mu[i] = moments( contours[i], false ); 
			mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		}


		// Start processing the contour data

		// Find Three repeatedly enclosed contours A,B,C
		// NOTE: 1. Contour enclosing other contours is assumed to be the three Alignment markings of the QR code.
		// 2. Alternately, the Ratio of areas of the "concentric" squares can also be used for identifying base Alignment markers.
		// The below demonstrates the first method
		
		for( int i = 0; i < contours.size(); i++ )
		{
			int k=i;
			int c=0;

			while(hierarchy[k][2] != -1)
			{
				k = hierarchy[k][2] ;
				c = c+1;
			}
			if(hierarchy[k][2] != -1)
			c = c+1;

			if (c >= 5)
			{	
				if (mark == 0)		A = i;
				else if  (mark == 1)	B = i;		// i.e., A is already found, assign current contour to B
				else if  (mark == 2)	C = i;		// i.e., A and B are already found, assign current contour to C
				mark = mark + 1 ;
			}
		} 

		
		if (mark >= 2)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
		{
			// We have found the 3 markers for the QR code; Now we need to determine which of them are 'top', 'right' and 'bottom' markers

			// Determining the 'top' marker
			// Vertex of the triangle NOT involved in the longest side is the 'outlier'

			AB = cv_distance(mc[A],mc[B]);
			BC = cv_distance(mc[B],mc[C]); 
			CA = cv_distance(mc[C],mc[A]);
			
			if ( AB > BC && AB > CA )
			{
				outlier = C; median1=A; median2=B;
			}
			else if ( CA > AB && CA > BC )
			{
				outlier = B; median1=A; median2=C;
			}
			else if ( BC > AB && BC > CA )
			{
				outlier = A;  median1=B; median2=C;
			}
						
			top = outlier;							// The obvious choice
		
			dist = cv_lineEquation(mc[median1], mc[median2], mc[outlier]);	// Get the Perpendicular distance of the outlier from the longest side			
			slope = cv_lineSlope(mc[median1], mc[median2],align);		// Also calculate the slope of the longest side
			
			// Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
			// Determine the 'right' and 'bottom' markers

			if (align == 0)
			{
				bottom = median1;
				right = median2;
			}
			else if (slope < 0 && dist < 0 )		// Orientation - North
			{
				bottom = median1;
				right = median2;
				orientation = CV_QR_NORTH;
			}	
			else if (slope > 0 && dist < 0 )		// Orientation - East
			{
				right = median1;
				bottom = median2;
				orientation = CV_QR_EAST;
			}
			else if (slope < 0 && dist > 0 )		// Orientation - South			
			{
				right = median1;
				bottom = median2;
				orientation = CV_QR_SOUTH;
			}

			else if (slope > 0 && dist > 0 )		// Orientation - West
			{
				bottom = median1;
				right = median2;
				orientation = CV_QR_WEST;
			}
	
			
			// To ensure any unintended values do not sneak up when QR code is not present
			float area_top,area_right, area_bottom;
			
			if( top < contours.size() && right < contours.size() && bottom < contours.size() && contourArea(contours[top]) > 10 && contourArea(contours[right]) > 10 && contourArea(contours[bottom]) > 10 )
			{

				vector<Point2f> L,M,O, tempL,tempM,tempO;
				Point2f N;	

				vector<Point2f> src,dst;		// src - Source Points basically the 4 end co-ordinates of the overlay image
												// dst - Destination Points to transform overlay image	

				Mat warp_matrix;

				cv_getVertices(contours,top,slope,tempL);
				cv_getVertices(contours,right,slope,tempM);
				cv_getVertices(contours,bottom,slope,tempO);

				cv_updateCornerOr(orientation, tempL, L); 			// Re-arrange marker corners w.r.t orientation of the QR code
				cv_updateCornerOr(orientation, tempM, M); 			// Re-arrange marker corners w.r.t orientation of the QR code
				cv_updateCornerOr(orientation, tempO, O); 			// Re-arrange marker corners w.r.t orientation of the QR code

				int iflag = getIntersectionPoint(M[1],M[2],O[3],O[2],N);

			
				src.push_back(L[0]);
				src.push_back(M[1]);
				src.push_back(N);
				src.push_back(O[3]);
	
				dst.push_back(Point2f(0,0));
				dst.push_back(Point2f(qr.cols,0));
				dst.push_back(Point2f(qr.cols, qr.rows));
				dst.push_back(Point2f(0, qr.rows));

				if (src.size() == 4 && dst.size() == 4 )			// Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
				{
					warp_matrix = getPerspectiveTransform(src, dst);
					warpPerspective(image, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
					copyMakeBorder( qr_raw, qr, 10, 10, 10, 10,BORDER_CONSTANT, Scalar(255,255,255) );
					
					cvtColor(qr,qr_gray,CV_RGB2GRAY);
					threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);
					
					//threshold(qr_gray, qr_thres, 0, 255, CV_THRESH_OTSU);
					//for( int d=0 ; d < 4 ; d++){	src.pop_back(); dst.pop_back(); }
				}
	
				//Draw contours on the image
				drawContours( image, contours, top , Scalar(255,200,0), 2, 8, hierarchy, 0 );
				drawContours( image, contours, right , Scalar(0,0,255), 2, 8, hierarchy, 0 );
				drawContours( image, contours, bottom , Scalar(255,0,100), 2, 8, hierarchy, 0 );

				// Insert Debug instructions here
				if(DBG==1)
				{
					// Debug Prints
					// Visualizations for ease of understanding
					if (slope > 5)
						circle( traces, Point(10,20) , 5 ,  Scalar(0,0,255), -1, 8, 0 );
					else if (slope < -5)
						circle( traces, Point(10,20) , 5 ,  Scalar(255,255,255), -1, 8, 0 );
						
					// Draw contours on Trace image for analysis	
					drawContours( traces, contours, top , Scalar(255,0,100), 1, 8, hierarchy, 0 );
					drawContours( traces, contours, right , Scalar(255,0,100), 1, 8, hierarchy, 0 );
					drawContours( traces, contours, bottom , Scalar(255,0,100), 1, 8, hierarchy, 0 );

					// Draw points (4 corners) on Trace image for each Identification marker	
					circle( traces, L[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( traces, L[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( traces, L[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( traces, L[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					circle( traces, M[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( traces, M[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( traces, M[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( traces, M[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					circle( traces, O[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( traces, O[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( traces, O[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( traces, O[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					// Draw point of the estimated 4th Corner of (entire) QR Code
					circle( traces, N, 2,  Scalar(255,255,255), -1, 8, 0 );

					// Draw the lines used for estimating the 4th Corner of QR Code
					line(traces,M[1],N,Scalar(0,0,255),1,8,0);
					line(traces,O[3],N,Scalar(0,0,255),1,8,0);


					// Show the Orientation of the QR Code wrt to 2D Image Space
					int fontFace = FONT_HERSHEY_PLAIN;
					 
					if(orientation == CV_QR_NORTH)
					{
						putText(traces, "NORTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientation == CV_QR_EAST)
					{
						putText(traces, "EAST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientation == CV_QR_SOUTH)
					{
						putText(traces, "SOUTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientation == CV_QR_WEST)
					{
						putText(traces, "WEST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}

					// Debug Prints
				}

			}
		}
	
		imshow ( "Image", image );
		imshow ( "Traces", traces );
		imshow ( "QR code", qr_thres );

		key = waitKey(1);	// OPENCV: wait for 1ms before accessing next frame

	}	// End of 'while' loop

	return 0;
}

// End of Main Loop
//--------------------------------------------------------------------------------------


// Routines used in Main loops

// Function: Routine to get Distance between two points
// Description: Given 2 points, the function returns the distance

float cv_distance(Point2f P, Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)) ; 
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//	  calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f L, Point2f M, Point2f J)
{
	float a,b,c,pdist;

	a = -((M.y - L.y) / (M.x - L.x));
	b = 1.0;
	c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;
	
	// Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

	pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
	return pdist;
}

// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
//	  indicates the line is vertical and the slope is infinity.

float cv_lineSlope(Point2f L, Point2f M, int& alignement)
{
	float dx,dy;
	dx = M.x - L.x;
	dy = M.y - L.y;
	
	if ( dy != 0)
	{	 
		alignement = 1;
		return (dy / dx);
	}
	else				// Make sure we are not dividing by zero; so use 'alignement' flag
	{	 
		alignement = 0;
		return 0.0;
	}
}



// Function: Routine to calculate 4 Corners of the Marker in Image Space using Region partitioning
// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
//	The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
//	exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
//	4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
//	every contour point in that region, the farthest point is deemed as the vertex of that region. Calculating
//	for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
{
	Rect box;
	box = boundingRect( contours[c_id]);
	
	Point2f M0,M1,M2,M3;
	Point2f A, B, C, D, W, X, Y, Z;

	A =  box.tl();
	B.x = box.br().x;
	B.y = box.tl().y;
	C = box.br();
	D.x = box.tl().x;
	D.y = box.br().y;


	W.x = (A.x + B.x) / 2;
	W.y = A.y;

	X.x = B.x;
	X.y = (B.y + C.y) / 2;

	Y.x = (C.x + D.x) / 2;
	Y.y = C.y;

	Z.x = D.x;
	Z.y = (D.y + A.y) / 2;

	float dmax[4];
	dmax[0]=0.0;
	dmax[1]=0.0;
	dmax[2]=0.0;
	dmax[3]=0.0;

	float pd1 = 0.0;
	float pd2 = 0.0;

	if (slope > 5 || slope < -5 )
	{

	    for( int i = 0; i < contours[c_id].size(); i++ )
	    {
		pd1 = cv_lineEquation(C,A,contours[c_id][i]);	// Position of point w.r.t the diagonal AC 
		pd2 = cv_lineEquation(B,D,contours[c_id][i]);	// Position of point w.r.t the diagonal BD

		if((pd1 >= 0.0) && (pd2 > 0.0))
		{
		    cv_updateCorner(contours[c_id][i],W,dmax[1],M1);
		}
		else if((pd1 > 0.0) && (pd2 <= 0.0))
		{
		    cv_updateCorner(contours[c_id][i],X,dmax[2],M2);
		}
		else if((pd1 <= 0.0) && (pd2 < 0.0))
		{
		    cv_updateCorner(contours[c_id][i],Y,dmax[3],M3);
		}
		else if((pd1 < 0.0) && (pd2 >= 0.0))
		{
		    cv_updateCorner(contours[c_id][i],Z,dmax[0],M0);
		}
		else
		    continue;
             }
	}
	else
	{
		int halfx = (A.x + B.x) / 2;
		int halfy = (A.y + D.y) / 2;

		for( int i = 0; i < contours[c_id].size(); i++ )
		{
			if((contours[c_id][i].x < halfx) && (contours[c_id][i].y <= halfy))
			{
			    cv_updateCorner(contours[c_id][i],C,dmax[2],M0);
			}
			else if((contours[c_id][i].x >= halfx) && (contours[c_id][i].y < halfy))
			{
			    cv_updateCorner(contours[c_id][i],D,dmax[3],M1);
			}
			else if((contours[c_id][i].x > halfx) && (contours[c_id][i].y >= halfy))
			{
			    cv_updateCorner(contours[c_id][i],A,dmax[0],M2);
			}
			else if((contours[c_id][i].x <= halfx) && (contours[c_id][i].y > halfy))
			{
			    cv_updateCorner(contours[c_id][i],B,dmax[1],M3);
			}
	    	}
	}

	quad.push_back(M0);
	quad.push_back(M1);
	quad.push_back(M2);
	quad.push_back(M3);
	
}

// Function: Compare a point if it more far than previously recorded farthest distance
// Description: Farthest Point detection using reference point and baseline distance
void cv_updateCorner(Point2f P, Point2f ref , float& baseline,  Point2f& corner)
{
    float temp_dist;
    temp_dist = cv_distance(P,ref);

    if(temp_dist > baseline)
    {
        baseline = temp_dist;			// The farthest distance is the new baseline
        corner = P;						// P is now the farthest point
    }
	
}

// Function: Sequence the Corners wrt to the orientation of the QR Code
void cv_updateCornerOr(int orientation, vector<Point2f> IN,vector<Point2f> &OUT)
{
	Point2f M0,M1,M2,M3;
    	if(orientation == CV_QR_NORTH)
	{
		M0 = IN[0];
		M1 = IN[1];
	 	M2 = IN[2];
		M3 = IN[3];
	}
	else if (orientation == CV_QR_EAST)
	{
		M0 = IN[1];
		M1 = IN[2];
	 	M2 = IN[3];
		M3 = IN[0];
	}
	else if (orientation == CV_QR_SOUTH)
	{
		M0 = IN[2];
		M1 = IN[3];
	 	M2 = IN[0];
		M3 = IN[1];
	}
	else if (orientation == CV_QR_WEST)
	{
		M0 = IN[3];
		M1 = IN[0];
	 	M2 = IN[1];
		M3 = IN[2];
	}

	OUT.push_back(M0);
	OUT.push_back(M1);
	OUT.push_back(M2);
	OUT.push_back(M3);
}

// Function: Get the Intersection Point of the lines formed by sets of two points
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
{
    Point2f p = a1;
    Point2f q = b1;
    Point2f r(a2-a1);
    Point2f s(b2-b1);

    if(cross(r,s) == 0) {return false;}

    float t = cross(q-p,s)/cross(r,s);

    intersection = p + t*r;
    return true;
}

float cross(Point2f v1,Point2f v2)
{
    return v1.x*v2.y - v1.y*v2.x;
}

// EOF





//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// PROTOTYPE 2 /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "QRReader.cpp"
using namespace cv;

static void help()
{
    printf("\nThis program demonstrates OpenCV drawing and text output functions.\n"
    "Usage:\n"
    "   ./drawing\n");
}
static Scalar randomColor(RNG& rng)
{
    int icolor = (unsigned)rng;
    return Scalar(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}

int main()
{

	//////////////////////////////////
	////////// BOUCLE VIDEO //////////
	//////////////////////////////////


    int c;
    // allocate memory for an image
    IplImage *img;
    // capture from video device #1
    CvCapture* capture = cvCaptureFromCAM(0);
    // create a window to display the images
    cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
    // position the window
    cvMoveWindow("mainWin", 5, 5);
	CQRReader qr = CQRReader();

    while(1)
    {
		Mat image;
		Mat imgBW;
        // retrieve the captured frame
        img=cvQueryFrame(capture);
		image = img;

		cvtColor(image, imgBW, CV_BGR2GRAY);
		threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);

		bool found = qr.Find(imgBW);
		if(found)
			qr.DrawFinders(imgBW);

        // show the image in the window
        cvShowImage("mainWin", img );

        // wait 10 ms for a key to be pressed
        c=cvWaitKey(10);

        // escape key terminates program
        if(c == 27)
        break;
    }
    return 0;
	*/

	/*
	//////////////////////////
	// Boucle capture vidéo //
	//////////////////////////

    cv::VideoCapture capture = VideoCapture(1);
	CQRReader qr = CQRReader();

	if(!capture.isOpened())
		printf("Unable to open camera");

	//Now, we create a simple loop that takes in images from the camera and passes them onto the QR code detecting class. It starts with declaring a few images and the loop:

	Mat image;
	Mat imgBW;
	while(true)
	{

	//Then, you grab an image from the camera:

		capture >> image;

	//Then you convert the image into a gray scale image and threshold it:

		cvtColor(image, imgBW, CV_BGR2GRAY);
		threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);

	//Now, you do the actual finding. If a code was actually found, draw it!
	// On check si il y a un QRCode à l'image
		bool found = qr.find(imgBW);
		if(found)
			qr.drawFinders(imgBW);

	//Finally, so whatever we got and wait for sometime:

		imshow("image", imgBW);
		waitKey(30);
	}

	//And end the main function:

	waitKey(0);

	return 0;
	
}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// PROTOTYPE 3 /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>
#define CONT vector<Point>

using namespace cv;
using namespace std;

struct FinderPattern{
    Point topleft;
    Point topright;
    Point bottomleft;
    FinderPattern(Point a, Point b, Point c) : topleft(a), topright(b), bottomleft(c) {}
};

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

Point getContourCentre(CONT& vec){
    double tempx = 0.0, tempy = 0.0;
    for(int i=0; i<vec.size(); i++){
        tempx += vec[i].x;
        tempy += vec[i].y;
    }
    return Point(tempx / (double)vec.size(), tempy / (double)vec.size());
}

bool isContourInsideContour(CONT& in, CONT& out){
    for(int i = 0; i<in.size(); i++){
        if(pointPolygonTest(out, in[i], false) <= 0) return false;
    }
    return true;
}

vector<CONT > findLimitedConturs(Mat contour, float minPix, float maxPix){
    vector<CONT > contours;
    vector<Vec4i> hierarchy;
    findContours(contour, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    cout<<"contours.size = "<<contours.size()<<endl;
    int m = 0; 
    while(m < contours.size()){
        if(contourArea(contours[m]) <= minPix){
            contours.erase(contours.begin() + m);
        }else if(contourArea(contours[m]) > maxPix){
            contours.erase(contours.begin() + m);
        }else ++ m;
    }
    cout<<"contours.size = "<<contours.size()<<endl;
    return contours;
}

vector<vector<CONT > > getContourPair(vector<CONT > &contours){
    vector<vector<CONT > > vecpair;
    vector<bool> bflag(contours.size(), false);

    for(int i = 0; i<contours.size() - 1; i++){
        if(bflag[i]) continue;
        vector<CONT > temp;
        temp.push_back(contours[i]);
        for(int j = i + 1; j<contours.size(); j++){
            if(isContourInsideContour(contours[j], contours[i])){
                temp.push_back(contours[j]);
                bflag[j] = true;
            }
        }
        if(temp.size() > 1){
            vecpair.push_back(temp);
        }
    }
    bflag.clear();
    for(int i=0; i<vecpair.size(); i++){
        sort(vecpair[i].begin(), vecpair[i].end(), compareContourAreas);
    }
    return vecpair;
}

void eliminatePairs(vector<vector<CONT > >& vecpair, double minRatio, double maxRatio){
    cout<<"maxRatio = "<<maxRatio<<endl;
    int m = 0; 
    bool flag = false;
    while(m < vecpair.size()){
        flag = false;
        if(vecpair[m].size() < 3){
            vecpair.erase(vecpair.begin() + m);
            continue;
        }
        for(int i=0; i<vecpair[m].size() - 1; i++){
            double area1 = contourArea(vecpair[m][i]);
            double area2 = contourArea(vecpair[m][i + 1]);
            if(area1 / area2 < minRatio || area1 / area2 > maxRatio){
                vecpair.erase(vecpair.begin() + m);
                flag = true;
                break;
            }
        }
        if(!flag){
            ++ m;
        }
    }
    if(vecpair.size() > 3){
        eliminatePairs(vecpair, minRatio, maxRatio * 0.9);
    }
}


double getDistance(Point a, Point b){
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

FinderPattern getFinderPattern(vector<vector<CONT > > &vecpair){
    Point pt1 = getContourCentre(vecpair[0][vecpair[0].size() - 1]);
    Point pt2 = getContourCentre(vecpair[1][vecpair[1].size() - 1]);
    Point pt3 = getContourCentre(vecpair[2][vecpair[2].size() - 1]);
    double d12 = getDistance(pt1, pt2);
    double d13 = getDistance(pt1, pt3);
    double d23 = getDistance(pt2, pt3);
    double x1, y1, x2, y2, x3, y3;
    double Max = max(d12, max(d13, d23));
    Point p1, p2, p3;
    if(Max == d12){
        p1 = pt1;
        p2 = pt2;
        p3 = pt3;
    }else if(Max == d13){
        p1 = pt1;
        p2 = pt3;
        p3 = pt2;
    }else if(Max == d23){
        p1 = pt2;
        p2 = pt3;
        p3 = pt1;
    }
    x1 = p1.x;
    y1 = p1.y;
    x2 = p2.x;
    y2 = p2.y;
    x3 = p3.x;
    y3 = p3.y;
    if(x1 == x2){
        if(y1 > y2){
            if(x3 < x1){
                return FinderPattern(p3, p2, p1);
            }else{
                return FinderPattern(p3, p1, p2);
            }
        }else{
            if(x3 < x1){
                return FinderPattern(p3, p1, p2);
            }else{
                return FinderPattern(p3, p2, p1);
            }
        }
    }else{
        double newy = (y2 - y1) / (x2 - x1) * x3 + y1 - (y2 - y1) / (x2 - x1) * x1;
        if(x1 > x2){
            if(newy < y3){
                return FinderPattern(p3, p2, p1);
            }else{
                return FinderPattern(p3, p1, p2);
            }
        }else{
            if(newy < y3){
                return FinderPattern(p3, p1, p2);
            }else{
                return FinderPattern(p3, p2, p1);
            }
        }
    }
}


int main()
{
    //Mat ori=imread("tshirt.png");
    Mat ori=imread("bigbook.jpg");
    Mat gray;
    cvtColor (ori,gray,CV_BGR2GRAY);

    Mat pcanny;
    gray.copyTo(pcanny);
    Canny( pcanny, pcanny, 50, 150, 3 );

    Mat bin;
    threshold(gray, bin, 0, 255, CV_THRESH_OTSU);
    Mat contour;
    bin.copyTo(contour);

    vector<CONT > contours;
    contours = findLimitedConturs(contour, 8.00, 0.2 * ori.cols * ori.rows);
*/
/*
    Mat drawing;
    ori.copyTo(drawing);
    for( int i = 0; i< contours.size(); i++ ){
        int r = (rand() + 125)%255;
        int g = (rand() + 32)%255;
        int b = (rand() + 87)%255;
       drawContours( drawing, contours, i, CV_RGB(r, g, b), 1);
    }
    imshow("contours", drawing);
*/
	/*
    if(!contours.empty()) sort(contours.begin(), contours.end(), compareContourAreas);
    vector<vector<CONT > > vecpair = getContourPair(contours);
    eliminatePairs(vecpair, 1.0, 10.0);
    cout<<"there are "<<vecpair.size()<<" pairs left!!"<<endl;

    FinderPattern fPattern = getFinderPattern(vecpair);
    cout<<"topleft = "<<fPattern.topleft.x<<", "<<fPattern.topleft.y<<endl
        <<"topright = "<<fPattern.topright.x<<", "<<fPattern.topright.y<<endl
        <<"bottomleft = "<<fPattern.bottomleft.x<<", "<<fPattern.bottomleft.y<<endl;
    Mat drawing;
    ori.copyTo(drawing);

    circle(drawing, fPattern.topleft, 3, CV_RGB(255,0,0), 2, 8, 0);
    circle(drawing, fPattern.topright, 3, CV_RGB(0,255,0), 2, 8, 0);
    circle(drawing, fPattern.bottomleft, 3, CV_RGB(0,0,255), 2, 8, 0);

    vector<Point2f> vecsrc;
    vector<Point2f> vecdst;
    vecsrc.push_back(fPattern.topleft);
    vecsrc.push_back(fPattern.topright);
    vecsrc.push_back(fPattern.bottomleft);
    vecdst.push_back(Point2f(20, 20));
    vecdst.push_back(Point2f(120, 20));
    vecdst.push_back(Point2f(20, 120));
    Mat affineTrans = getAffineTransform(vecsrc, vecdst);
    Mat warped;
    warpAffine(ori, warped, affineTrans, ori.size());
    Mat qrcode_color = warped(Rect(0, 0, 140, 140));
    Mat qrcode_gray;
    cvtColor (qrcode_color,qrcode_gray,CV_BGR2GRAY);
    Mat qrcode_bin;
    threshold(qrcode_gray, qrcode_bin, 0, 255, CV_THRESH_OTSU);

    imshow("binary", bin);
    imshow("canny", pcanny);
    imshow("finder patterns", drawing);
    imshow("binaried qr code", qrcode_bin);

    waitKey();
    return 0;
}

*/