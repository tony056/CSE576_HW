#include "mainwindow.h"
#include "math.h"
#include "ui_mainwindow.h"
#include <QtGui>
#include "Matrix.h"
#include <float.h>

/*******************************************************************************
    The following are helper routines with code already written.
    The routines you'll need to write for the assignment are below.
*******************************************************************************/

/*******************************************************************************
Draw detected Harris corners
    cornerPts - corner points
    numCornerPts - number of corner points
    imageDisplay - image used for drawing

    Draws a red cross on top of detected corners
*******************************************************************************/
void MainWindow::DrawCornerPoints(CIntPt *cornerPts, int numCornerPts, QImage &imageDisplay)
{
   int i;
   int r, c, rd, cd;
   int w = imageDisplay.width();
   int h = imageDisplay.height();

   for(i=0;i<numCornerPts;i++)
   {
       c = (int) cornerPts[i].m_X;
       r = (int) cornerPts[i].m_Y;

       for(rd=-2;rd<=2;rd++)
           if(r+rd >= 0 && r+rd < h && c >= 0 && c < w)
               imageDisplay.setPixel(c, r + rd, qRgb(255, 0, 0));

       for(cd=-2;cd<=2;cd++)
           if(r >= 0 && r < h && c + cd >= 0 && c + cd < w)
               imageDisplay.setPixel(c + cd, r, qRgb(255, 0, 0));
   }
}

/*******************************************************************************
Compute corner point descriptors
    image - input image
    cornerPts - array of corner points
    numCornerPts - number of corner points

    If the descriptor cannot be computed, i.e. it's too close to the boundary of
    the image, its descriptor length will be set to 0.

    I've implemented a very simple 8 dimensional descriptor.  Feel free to
    improve upon this.
*******************************************************************************/
void MainWindow::ComputeDescriptors(QImage image, CIntPt *cornerPts, int numCornerPts)
{
    int r, c, cd, rd, i, j;
    int w = image.width();
    int h = image.height();
    double *buffer = new double [w*h];
    QRgb pixel;

    // Descriptor parameters
    double sigma = 2.0;
    int rad = 4;

    // Computer descriptors from green channel
    for(r=0;r<h;r++)
       for(c=0;c<w;c++)
        {
            pixel = image.pixel(c, r);
            buffer[r*w + c] = (double) qGreen(pixel);
        }

    // Blur
    GaussianBlurImage(buffer, w, h, sigma);

    // Compute the desciptor from the difference between the point sampled at its center
    // and eight points sampled around it.
    for(i=0;i<numCornerPts;i++)
    {
        int c = (int) cornerPts[i].m_X;
        int r = (int) cornerPts[i].m_Y;

        if(c >= rad && c < w - rad && r >= rad && r < h - rad)
        {
            double centerValue = buffer[(r)*w + c];
            int j = 0;

            for(rd=-1;rd<=1;rd++)
                for(cd=-1;cd<=1;cd++)
                    if(rd != 0 || cd != 0)
                {
                    cornerPts[i].m_Desc[j] = buffer[(r + rd*rad)*w + c + cd*rad] - centerValue;
                    j++;
                }

            cornerPts[i].m_DescSize = DESC_SIZE;
        }
        else
        {
            cornerPts[i].m_DescSize = 0;
        }
    }

    delete [] buffer;
}

/*******************************************************************************
Draw matches between images
    matches - matching points
    numMatches - number of matching points
    image1Display - image to draw matches
    image2Display - image to draw matches

    Draws a green line between matches
*******************************************************************************/
void MainWindow::DrawMatches(CMatches *matches, int numMatches, QImage &image1Display, QImage &image2Display)
{
    int i;
    // Show matches on image
    QPainter painter;
    painter.begin(&image1Display);
    QColor green(0, 250, 0);
    QColor red(250, 0, 0);

    for(i=0;i<numMatches;i++)
    {
        painter.setPen(green);
        painter.drawLine((int) matches[i].m_X1, (int) matches[i].m_Y1, (int) matches[i].m_X2, (int) matches[i].m_Y2);
        painter.setPen(red);
        painter.drawEllipse((int) matches[i].m_X1-1, (int) matches[i].m_Y1-1, 3, 3);
    }

    QPainter painter2;
    painter2.begin(&image2Display);
    painter2.setPen(green);

    for(i=0;i<numMatches;i++)
    {
        painter2.setPen(green);
        painter2.drawLine((int) matches[i].m_X1, (int) matches[i].m_Y1, (int) matches[i].m_X2, (int) matches[i].m_Y2);
        painter2.setPen(red);
        painter2.drawEllipse((int) matches[i].m_X2-1, (int) matches[i].m_Y2-1, 3, 3);
    }

}


/*******************************************************************************
Given a set of matches computes the "best fitting" homography
    matches - matching points
    numMatches - number of matching points
    h - returned homography
    isForward - direction of the projection (true = image1 -> image2, false = image2 -> image1)
*******************************************************************************/
bool MainWindow::ComputeHomography(CMatches *matches, int numMatches, double h[3][3], bool isForward)
{
    int error;
    int nEq=numMatches*2;

    dmat M=newdmat(0,nEq,0,7,&error);
    dmat a=newdmat(0,7,0,0,&error);
    dmat b=newdmat(0,nEq,0,0,&error);

    double x0, y0, x1, y1;

    for (int i=0;i<nEq/2;i++)
    {
        if(isForward == false)
        {
            x0 = matches[i].m_X1;
            y0 = matches[i].m_Y1;
            x1 = matches[i].m_X2;
            y1 = matches[i].m_Y2;
        }
        else
        {
            x0 = matches[i].m_X2;
            y0 = matches[i].m_Y2;
            x1 = matches[i].m_X1;
            y1 = matches[i].m_Y1;
        }


        //Eq 1 for corrpoint
        M.el[i*2][0]=x1;
        M.el[i*2][1]=y1;
        M.el[i*2][2]=1;
        M.el[i*2][3]=0;
        M.el[i*2][4]=0;
        M.el[i*2][5]=0;
        M.el[i*2][6]=(x1*x0*-1);
        M.el[i*2][7]=(y1*x0*-1);

        b.el[i*2][0]=x0;
        //Eq 2 for corrpoint
        M.el[i*2+1][0]=0;
        M.el[i*2+1][1]=0;
        M.el[i*2+1][2]=0;
        M.el[i*2+1][3]=x1;
        M.el[i*2+1][4]=y1;
        M.el[i*2+1][5]=1;
        M.el[i*2+1][6]=(x1*y0*-1);
        M.el[i*2+1][7]=(y1*y0*-1);

        b.el[i*2+1][0]=y0;

    }
    int ret=solve_system (M,a,b);
    if (ret!=0)
    {
        freemat(M);
        freemat(a);
        freemat(b);

        return false;
    }
    else
    {
        h[0][0]= a.el[0][0];
        h[0][1]= a.el[1][0];
        h[0][2]= a.el[2][0];

        h[1][0]= a.el[3][0];
        h[1][1]= a.el[4][0];
        h[1][2]= a.el[5][0];

        h[2][0]= a.el[6][0];
        h[2][1]= a.el[7][0];
        h[2][2]= 1;
    }

    freemat(M);
    freemat(a);
    freemat(b);

    return true;
}


/*******************************************************************************
*******************************************************************************
*******************************************************************************

    The routines you need to implement are below

*******************************************************************************
*******************************************************************************
*******************************************************************************/


/*******************************************************************************
Blur a single channel floating point image with a Gaussian.
    image - input and output image
    w - image width
    h - image height
    sigma - standard deviation of Gaussian

    This code should be very similar to the code you wrote for assignment 1.
*******************************************************************************/
void NormalizeKernel(double *kernel, int kernelWidth, int kernelHeight)
{
    double denom = 0.000001; int i;
    for(i=0; i<kernelWidth*kernelHeight; i++)
        denom += kernel[i];
    for(i=0; i<kernelWidth*kernelHeight; i++)
        kernel[i] /= denom;
}



void Convolution(double* image, double *kernel, int kernelWidth, int kernelHeight, bool add, int width, int height)
{
    int radius_w = kernelWidth / 2;
    int radius_h = kernelHeight / 2;
    // qDebug("w: %d, h: %d", radius_w, radius_h);
    double *newImage = new double[width*height];
    // Image = new double* [width*height];
    // for (int i = 0; i < width*height; i++)
    //     newImage[i] = new double[3];
    for(int r = 0; r < height; r++){
        for(int c = 0; c < width; c++){

            double rgb[3];
            rgb[0] = rgb[1] = rgb[2] = (add == true) ? 128.0 : 0.0;

            for(int rd = -radius_h; rd <= radius_h; rd++){
                for(int cd = -radius_w; cd <=radius_w; cd++){

                    int pixel = (r + rd) * width + c + cd;
                    double weight = kernel[(rd + radius_h)*kernelWidth + cd + radius_w];
                    // int currentR = ((r + rd < 0 || r + rd >= height) || (c + cd < 0 || c + cd >= width)) ? 0 : image[pixel][0];
                    int currentG = ((r + rd < 0 || r + rd >= height) || (c + cd < 0 || c + cd >= width)) ? 0 : image[pixel];
                    // int currentB = ((r + rd < 0 || r + rd >= height) || (c + cd < 0 || c + cd >= width)) ? 0 : image[pixel][2];
                    // rgb[0] += weight*currentR;
                    rgb[0] += weight*currentG;
                    // rgb[2] += weight*currentB;

                }
            }

            newImage[r * width + c] = rgb[0];
            // newImage[r * width + c][1] = rgb[1];
            // newImage[r * width + c][2] = rgb[2];
        }
    }
    //refractor memory copy part
    for(int r = 0; r < height; r++){
        for(int c = 0; c < width; c++){
            image[r * width + c] = newImage[r * width + c];
            // image[r * width + c][1] = newImage[r * width + c][1];
            // image[r * width + c][2] = newImage[r * width + c][2];
        }
    }
}

void Sobel(double *image, int direction, int w, int h)
{
    double *sobel_x = new double[9];
    double *sobel_y = new double[9];
    sobel_x[0] = -1;
    sobel_x[1] = 0;
    sobel_x[2] = 1;
    sobel_x[3] = -2;
    sobel_x[4] = 0;
    sobel_x[5] = 2;
    sobel_x[6] = -1;
    sobel_x[7] = 0;
    sobel_x[8] = 1;

    sobel_y[0] = 1;
    sobel_y[1] = 2;
    sobel_y[2] = 1;
    sobel_y[3] = 0;
    sobel_y[4] = 0;
    sobel_y[5] = 0;
    sobel_y[6] = -1;
    sobel_y[7] = -2;
    sobel_y[8] = -1;

    if(direction == 1)
        Convolution(image, sobel_y, 3, 3, false, w, h);
    else if(direction == 0)
        Convolution(image, sobel_x, 3, 3, false, w, h);
}

void MainWindow::GaussianBlurImage(double *image, int w, int h, double sigma)
{
    // Add your code here

    // To access the pixel (c,r), use image[r*width + c].
    if(sigma == 0)
        return;
    int radius = (int)(ceil(3 * sigma));
    int size = 2 * radius + 1;
    double *kernel = new double[size*size];
    double s = 2 * sigma * sigma;
    for(int i = -radius; i <= radius; i++){
        for(int j = -radius; j <= radius; j++){
            double r = sqrt(i * i + j * j);
            double val = (exp(-(r * r) / s)) / (s * M_PI);
            kernel[(i + radius) * size + j + radius] = val;
            // qDebug("val %d, %d: %f", i, j, kernel[(i + radius) * size + j + radius]);
        }
    }
    //NormalizeKernel
    NormalizeKernel(kernel, size, size);
    Convolution(image, kernel, size, size, false, w, h);
    //Convolution
}



void multiply(double *image1, double *image2, double *result, int width, int height)
{

    for(int i = 0; i < width * height; i++){
        result[i] = image1[i] * image2[i];
    }
}

void ComputeHarrisResponse(double *ix2, double *iy2, double *ixy, double *result, int w, int h)
{
    for(int i = 0; i < w * h; i++){
        // double ix2 = ix[i] * ix[i];
        // double iy2 = iy[i] * iy[i];
        // double ixy = ix[i] * iy[i];
        // double iyx = iy[i] * ix[i];
        double det = ix2[i] * iy2[i] - ixy[i] * ixy[i];
        double trace = ix2[i] + iy2[i];
        result[i] = det / trace; //no idea
        // qDebug("result %d = %f %f %f", i, result[i], det, trace);
    }
}

bool IsLocalMaximum(double *imageOfHR, int r, int c, int w, int h)
{
    int rd = 1;
    double current = imageOfHR[r * w + c];
    for(int i = -rd + r; i <= rd + r; i++){
        for(int j = -rd + c; j <= rd + c; j++){
            if(i < 0 || i >= h || j < 0 || j >= w)
                continue;
            if(imageOfHR[i * w + j] > current){
                return false;
            }
        }
    }
    return true;
}

/*******************************************************************************
Detect Harris corners.
    image - input image
    sigma - standard deviation of Gaussian used to blur corner detector
    thres - Threshold for detecting corners
    cornerPts - returned corner points
    numCornerPts - number of corner points returned
    imageDisplay - image returned to display (for debugging)
*******************************************************************************/
void MainWindow::HarrisCornerDetector(QImage image, double sigma, double thres, CIntPt **cornerPts, int &numCornerPts, QImage &imageDisplay)
{
    int r, c;
    int w = image.width();
    int h = image.height();
    double *buffer = new double [w*h];
    QRgb pixel;

    numCornerPts = 0;

    // Compute the corner response using just the green channel
    for(r=0;r<h;r++)
       for(c=0;c<w;c++)
        {
            pixel = image.pixel(c, r);

            buffer[r*w + c] = (double) qGreen(pixel);
        }

    // Write your Harris corner detection code here.
    double *ix = new double[w * h];
    double *iy = new double[w * h];
    double *iy2 = new double[w * h];
    double *ix2 = new double[w * h];
    double *ixy = new double[w * h];
    double *result = new double[w * h];
    for(int i = 0; i < w * h; i++){
        ix[i] = buffer[i];
        iy[i] = buffer[i];
    }

    Sobel(ix, 0, w, h);
    Sobel(iy, 1, w, h);
    multiply(ix, ix, ix2, w, h);
    multiply(iy, iy, iy2, w, h);
    multiply(ix, iy, ixy, w, h);
    GaussianBlurImage(ix2, w, h, sigma);
    GaussianBlurImage(iy2, w, h, sigma);
    GaussianBlurImage(ixy, w, h, sigma);

    ComputeHarrisResponse(ix2, iy2, ixy, result, w, h);
    // int count = 0;
    for(int i = 0; i < w * h; i++){
        // qDebug("result %d = %f", i, result[i]);
        if(result[i] > thres)
            numCornerPts++;
    }
    qDebug("count: %d", numCornerPts);
    *cornerPts = new CIntPt[numCornerPts];
    int index = 0;
    for(int r = 0; r < h; r++){
        for(int c = 0; c < w; c++){
            if(result[r * w + c] > thres && IsLocalMaximum(result, r, c, w, h) && index < numCornerPts){
                (*cornerPts)[index].m_X = c;
                (*cornerPts)[index].m_Y = r;
                index++;
            }
        }
    }
    // Once you uknow the number of corner points allocate an array as follows:
    // *cornerPts = new CIntPt [numCornerPts];
    // Access the values using: (*cornerPts)[i].m_X = 5.0;
    //
    // The position of the corner point is (m_X, m_Y)
    // The descriptor of the corner point is stored in m_Desc
    // The length of the descriptor is m_DescSize, if m_DescSize = 0, then it is not valid.

    // Once you are done finding the corner points, display them on the image
    DrawCornerPoints(*cornerPts, numCornerPts, imageDisplay);

    delete [] buffer;
    delete [] ix;
    delete [] iy;
    delete [] ix2;
    delete [] iy2;
    delete [] ixy;
    delete [] result;
}


/*******************************************************************************
Find matching corner points between images.
    image1 - first input image
    cornerPts1 - corner points corresponding to image 1
    numCornerPts1 - number of corner points in image 1
    image2 - second input image
    cornerPts2 - corner points corresponding to image 2
    numCornerPts2 - number of corner points in image 2
    matches - set of matching points to be returned
    numMatches - number of matching points returned
    image1Display - image used to display matches
    image2Display - image used to display matches
*******************************************************************************/
void MainWindow::MatchCornerPoints(QImage image1, CIntPt *cornerPts1, int numCornerPts1,
                             QImage image2, CIntPt *cornerPts2, int numCornerPts2,
                             CMatches **matches, int &numMatches, QImage &image1Display, QImage &image2Display)
{
    numMatches = 0;

    // Compute the descriptors for each corner point.
    // You can access the descriptor for each corner point using cornerPts1[i].m_Desc[j].
    // If cornerPts1[i].m_DescSize = 0, it was not able to compute a descriptor for that point
    ComputeDescriptors(image1, cornerPts1, numCornerPts1);
    ComputeDescriptors(image2, cornerPts2, numCornerPts2);

    for(int i = 0; i < numCornerPts1; i++){
        if(cornerPts1[i].m_DescSize > 0)
            numMatches++;
    }
    *matches = new CMatches [numMatches];
    int index = 0;
    // Add your code here for finding the best matches for each point.
    for(int i = 0; i < numCornerPts1; i++){
        if(cornerPts1[i].m_DescSize == 0)
            continue;
        double min_dist = DBL_MAX;
        for(int j = 0; j < numCornerPts2; j++){
            if(cornerPts2[j].m_DescSize == 0)
                continue;
            double dist = 0.0;
            for(int k = 0; k < cornerPts1[i].m_DescSize; k++){
                dist += fabs(cornerPts1[i].m_Desc[k] - cornerPts2[j].m_Desc[k]);
            }
            if(min_dist > dist){
                min_dist = dist;
                (*matches)[index].m_X1 = cornerPts1[i].m_X;
                (*matches)[index].m_Y1 = cornerPts1[i].m_Y;
                (*matches)[index].m_X2 = cornerPts2[j].m_X;
                (*matches)[index].m_Y2 = cornerPts2[j].m_Y;
            }
        }
        index++;
    }
    // Once you uknow the number of matches allocate an array as follows:
    // *matches = new CMatches [numMatches];
    //
    // The position of the corner point in iamge 1 is (m_X1, m_Y1)
    // The position of the corner point in image 2 is (m_X2, m_Y2)

    // Draw the matches
    DrawMatches(*matches, numMatches, image1Display, image2Display);
}

/*******************************************************************************
Project a point (x1, y1) using the homography transformation h
    (x1, y1) - input point
    (x2, y2) - returned point
    h - input homography used to project point
*******************************************************************************/
void MainWindow::Project(double x1, double y1, double &x2, double &y2, double h[3][3])
{
    // Add your code here.
    x2 = (h[0][0] * x1 + h[0][1] * y1 + h[0][2]) / (h[2][0] * x1 + h[2][1] * y1 + h[2][2]);
    y2 = (h[1][0] * x1 + h[1][1] * y1 + h[1][2]) / (h[2][0] * x1 + h[2][1] * y1 + h[2][2]);
}

/*******************************************************************************
Count the number of inliers given a homography.  This is a helper function for RANSAC.
    h - input homography used to project points (image1 -> image2
    matches - array of matching points
    numMatches - number of matchs in the array
    inlierThreshold - maximum distance between points that are considered to be inliers

    Returns the total number of inliers.
*******************************************************************************/
int MainWindow::ComputeInlierCount(double h[3][3], CMatches *matches, int numMatches, double inlierThreshold)
{
    // Add your code here.
    int count = 0;
    for(int i = 0; i < numMatches; i++){
        double xp = matches[i].m_X1;
        double yp = matches[i].m_Y1;
        Project(matches[i].m_X1, matches[i].m_Y1, xp, yp, h);
        double dist = fabs(xp - matches[i].m_X2) + fabs(yp - matches[i].m_Y2);
        if(dist < inlierThreshold)
            count++;
    }
    return count;
}


/*******************************************************************************
Compute homography transformation between images using RANSAC.
    matches - set of matching points between images
    numMatches - number of matching points
    numIterations - number of iterations to run RANSAC
    inlierThreshold - maximum distance between points that are considered to be inliers
    hom - returned homography transformation (image1 -> image2)
    homInv - returned inverse homography transformation (image2 -> image1)
    image1Display - image used to display matches
    image2Display - image used to display matches
*******************************************************************************/
void MainWindow::RANSAC(CMatches *matches, int numMatches, int numIterations, double inlierThreshold,
                        double hom[3][3], double homInv[3][3], QImage &image1Display, QImage &image2Display)
{
    // Add your code here.
    int maxRandomNum = 4;
    int *indexes = new int[maxRandomNum];
    int maxInlierCount = 0;
    for(int i = 0; i < numIterations; i++){
        int randomNum = 0;
        CMatches *selectedMatches = new CMatches[maxRandomNum];
        while(randomNum < maxRandomNum){
            int index = random() % numMatches;
            bool isSame = false;
            for(int j = 0; j < randomNum; j++){
                if(indexes[j] == index)
                    isSame = true;
            }
            if(!isSame){
                indexes[randomNum] = index;
                selectedMatches[randomNum] = matches[index];
                randomNum++;
            }
        }
        double h[3][3];
        // for(int k = 0; k < 3; k++)
        //     h[k] = new double[3];
        ComputeHomography(selectedMatches, maxRandomNum, h, true);
        int count = ComputeInlierCount(h, matches, numMatches, inlierThreshold);
        if(count > maxInlierCount){
            maxInlierCount = count;
            for(int k = 0; k < 3; k++){
                for(int l = 0; l < 3; l++)
                    hom[k][l] = h[k][l];
            }
        }
    }
    int index = 0;
    CMatches *inliers = new CMatches[maxInlierCount];
    for(int i = 0; i < numMatches; i++){
        double xp = matches[i].m_X1;
        double yp = matches[i].m_Y1;
        Project(matches[i].m_X1, matches[i].m_Y1, xp, yp, hom);
        double dist = fabs(xp - matches[i].m_X2) + fabs(yp - matches[i].m_Y2);
        if(dist < inlierThreshold && index < maxInlierCount)
            inliers[index++] = matches[i];
    }
    ComputeHomography(inliers, maxInlierCount, hom, true);
    ComputeHomography(inliers, maxInlierCount, homInv, false);
    // After you're done computing the inliers, display the corresponding matches.
    int numInliers = ComputeInlierCount(hom, inliers, maxInlierCount, inlierThreshold);
    DrawMatches(inliers, numInliers, image1Display, image2Display);
}

/*******************************************************************************
Bilinearly interpolate image (helper function for Stitch)
    image - input image
    (x, y) - location to interpolate
    rgb - returned color values

    You can just copy code from previous assignment.
*******************************************************************************/
bool MainWindow::BilinearInterpolation(QImage *image, double x, double y, double rgb[3])
{
    // Add your code here.

    return true;
}


/*******************************************************************************
Stitch together two images using the homography transformation
    image1 - first input image
    image2 - second input image
    hom - homography transformation (image1 -> image2)
    homInv - inverse homography transformation (image2 -> image1)
    stitchedImage - returned stitched image
*******************************************************************************/
void MainWindow::Stitch(QImage image1, QImage image2, double hom[3][3], double homInv[3][3], QImage &stitchedImage)
{
    // Width and height of stitchedImage
    int ws = 0;
    int hs = 0;

    // Add your code to compute ws and hs here.

    stitchedImage = QImage(ws, hs, QImage::Format_RGB32);
    stitchedImage.fill(qRgb(0,0,0));

    // Add you code to warp image1 and image2 to stitchedImage here.
}
