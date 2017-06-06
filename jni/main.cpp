#include "projectorCamera.h"
#include  "AstraData.h"

int  main(int argc, char* argv[])
{

	ProjectorCamera rsPeojection;
	rsPeojection.calibrationFixed();
	//rsPeojection.screenRoi = Rect(30,0,300,200);

	AstraData astradata;
	astradata.init();

	while (1)
	{
		astradata.getData();
		if (astradata.cvRawImg16U.empty() || astradata.cvBGRImg.empty())
			continue;

		Mat depthImg, colorImg;
		depthImg = astradata.cvRawImg16U.clone();
		colorImg = astradata.cvBGRImg.clone();
		depthImg.convertTo(depthImg, CV_32F);
		flip(colorImg, colorImg, 1);
		flip(depthImg, depthImg, 1);
		//imshow("colorImg", colorImg(rsPeojection.screenRoi));
		//waitKey(1);
		//continue;
		rsPeojection.processing(colorImg, depthImg);
	
	}
	return 0;
}