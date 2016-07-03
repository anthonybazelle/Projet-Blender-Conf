#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

using namespace cv;

class CQRReader
{
	public:
	
	CQRReader(void)
	{
	}


	~CQRReader(void)
	{
	}

	bool CQRReader::Find(Mat img)
	{
		//This function takes in an image to scan and returns true if it found a QR code. You can use the class functions to access information about most recent QR code (location, orientation, etc).
		//Earlier, I mentioned the the detector goes through each line. You could do that, but it turns out that scanning every third line makes it work quite as too. So I’ve introduced a new variable skipRows that you can tweak:

			int skipRows = 3;

		//Now, we initialize an array that maintains the state data and create a variable that keeps track of the current state. We also create a loop that iterates through the image.
			printf("TestFind");
			int stateCount[5] = {0};
			int currentState = 0;
			for(int row=skipRows-1; row<img.rows; row+=skipRows)
			{

		//Now, we initialize stateCount to zero and set the current state to zero. This is because you check if a row has the finder pattern. You need to erase state data from the previous rows.

				stateCount[0] = 0;
				stateCount[1] = 0;
				stateCount[2] = 0;
				stateCount[3] = 0;
				stateCount[4] = 0;
				currentState = 0;

		//Next, get a pointer to the current row’s raw pixels and start iterating across it:

				uchar* ptr = img.ptr<uchar>(row);
				for(int col=0; col<img.cols; col++)
				{

		//Now, we check if we’re at a black pixel. If we are and the state is one of being inside a white (state 1 and state 3), we move to the next state. And for either case (being inside white or black), we increment the stateCount of the ‘current’ state:

					if(ptr[col]<128)
					{
						// We're at a black pixel
						if((currentState & 0x1)==1)
						{
							// We were counting white pixels
							// So change the state now
							// W->B transition
							currentState++;
						}

						// Works for boths W->B and B->B
						stateCount[currentState]++;
					}

		//Now, if we’re at a white pixel:

					else
					{

		//If we were counting white pixels, simply increment the stateCount of the correct state:

						// We got to a white pixel...
						if((currentState & 0x1)==1)
						{
							// W->W change
							stateCount[currentState]++;
						}

		//If not, we need to check some conditions. This is slightly tricky:

						else
						{
							// ...but, we were counting black pixels
							if(currentState==4)
							{

		//So what we do is, check if the ratio is what we expected – 1:1:3:1:1

								// We found the 'white' area AFTER the finder patter
								// Do processing for it here
								if(CheckRatio(stateCount))
								{
									printf("On check le ratio");
		// And you know that you’re at a possible finder pattern. More checks need to be done to ensure this is an actual finder pattern. We’ll do that in the next post. So, for now, I’ll just leave a comment:
		// This is where we do some more checks
								}

		//We could also check if we found all the three finder patterns. If we did, return a true. But we’ll do that in the next part. But, if the ratio isn’t right, we need to do the switch I mentioned earlier:

								else
								{
									currentState = 3;
									stateCount[0] = stateCount[2];
									stateCount[1] = stateCount[3];
									stateCount[2] = stateCount[4];
									stateCount[3] = 1;
									stateCount[4] = 0;
									continue;
								}

		//And other than that, here’s what we gotta do:
								currentState = 0;
								stateCount[0] = 0;
								stateCount[1] = 0;
								stateCount[2] = 0;
								stateCount[3] = 0;
								stateCount[4] = 0;
							}
							else
							{
								// We still haven't go 'out' of the finder pattern yet
								// So increment the state
								// B->W transition
								currentState++;
								stateCount[currentState]++;
							}

		//Now, close all the loops and return ‘false’:

						}
					}
				}
			}
			return false;
	}

	// Permet de connaitre notre position par rapport au QR Code
	bool CQRReader::CheckRatio(int stateCount[])
	{
		int totalFinderSize = 0;
		for(int i=0; i<5; i++)
		{
			int count = stateCount[i];
			totalFinderSize += count;
			if(count==0)
				return false;
		}

		if(totalFinderSize<7)
			return false;

		// Calculate the size of one module
		int moduleSize = ceil(totalFinderSize / 7.0);
		int maxVariance = moduleSize/2;

		bool retVal= ((abs(moduleSize - (stateCount[0])) < maxVariance) &&
			(abs(moduleSize - (stateCount[1])) < maxVariance) &&
			(abs(3*moduleSize - (stateCount[2])) < 3*maxVariance) &&
			(abs(moduleSize - (stateCount[3])) < maxVariance) &&
			(abs(moduleSize - (stateCount[4])) < maxVariance));

		return retVal;
	}

	// On modélise en 3D
	void CQRReader::DrawFinders(Mat img)
	{
		printf("HELLO !!!");
	}

};