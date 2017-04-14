#include "mainwindow.h"
#include "math.h"
#include "ui_mainwindow.h"
#include <QtGui>
#include <iostream>
/***********************************************************************
  This is the only file you need to change for your assignment. The
  other files control the UI (in case you want to make changes.)
************************************************************************/

/***********************************************************************
  The first eight functions provide example code to help get you started
************************************************************************/


// Convert an image to grayscale
void MainWindow::BlackWhiteImage(QImage *image)
{
    for(int r=0;r<image->height();r++)
        for(int c=0;c<image->width();c++)
        {
            QRgb pixel = image->pixel(c, r);
            double red = (double) qRed(pixel);
            double green = (double) qGreen(pixel);
            double blue = (double) qBlue(pixel);

            // Compute intensity from colors - these are common weights
            double intensity = 0.3*red + 0.6*green + 0.1*blue;

            image->setPixel(c, r, qRgb( (int) intensity, (int) intensity, (int) intensity));
        }
}

// Add random noise to the image
void MainWindow::AddNoise(QImage *image, double mag, bool colorNoise)
{
    int noiseMag = mag*2;

    for(int r=0;r<image->height();r++)
        for(int c=0;c<image->width();c++)
        {
            QRgb pixel = image->pixel(c, r);
            int red = qRed(pixel), green = qGreen(pixel), blue = qBlue(pixel);

            // If colorNoise, add color independently to each channel
            if(colorNoise)
            {
                red += rand()%noiseMag - noiseMag/2;
                green += rand()%noiseMag - noiseMag/2;
                blue += rand()%noiseMag - noiseMag/2;
            }
            // otherwise add the same amount of noise to each channel
            else
            {
                int noise = rand()%noiseMag - noiseMag/2;
                red += noise; green += noise; blue += noise;
            }
            image->setPixel(c, r, qRgb(max(0, min(255, red)), max(0, min(255, green)), max(0, min(255, blue))));
        }
}

// Downsample the image by 1/2
void MainWindow::HalfImage(QImage &image)
{
    int w = image.width();
    int h = image.height();
    QImage buffer = image.copy();

    // Reduce the image size.
    image = QImage(w/2, h/2, QImage::Format_RGB32);

    // Copy every other pixel
    for(int r=0;r<h/2;r++)
        for(int c=0;c<w/2;c++)
             image.setPixel(c, r, buffer.pixel(c*2, r*2));
}

// Round float values to the nearest integer values and make sure the value lies in the range [0,255]
QRgb restrictColor(double red, double green, double blue)
{
    int r = (int)(floor(red+0.5));
    int g = (int)(floor(green+0.5));
    int b = (int)(floor(blue+0.5));

    return qRgb(max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)));
}

// Normalize the values of the kernel to sum-to-one
void NormalizeKernel(double *kernel, int kernelWidth, int kernelHeight)
{
    double denom = 0.000001; int i;
    for(i=0; i<kernelWidth*kernelHeight; i++)
        denom += kernel[i];
    for(i=0; i<kernelWidth*kernelHeight; i++)
        kernel[i] /= denom;
}

// Here is an example of blurring an image using a mean or box filter with the specified radius.
// This could be implemented using separable filters to make it much more efficient, but it's not done here.
// Note: This function is written using QImage form of the input image. But all other functions later use the double form
void MainWindow::MeanBlurImage(QImage *image, int radius)
{
    if(radius == 0)
        return;
    int size = 2*radius + 1; // This is the size of the kernel

    // Note: You can access the width and height using 'imageWidth' and 'imageHeight' respectively in the functions you write
    int w = image->width();
    int h = image->height();

    // Create a buffer image so we're not reading and writing to the same image during filtering.
    // This creates an image of size (w + 2*radius, h + 2*radius) with black borders (zero-padding)
    QImage buffer = image->copy(-radius, -radius, w + 2*radius, h + 2*radius);

    // Compute the kernel to convolve with the image
    double *kernel = new double [size*size];

    for(int i=0;i<size*size;i++)
        kernel[i] = 1.0;

    // Make sure kernel sums to 1
    NormalizeKernel(kernel, size, size);

    // For each pixel in the image...
    for(int r=0;r<h;r++)
    {
        for(int c=0;c<w;c++)
        {
            double rgb[3];
            rgb[0] = rgb[1] = rgb[2] = 0.0;

            // Convolve the kernel at each pixel
            for(int rd=-radius;rd<=radius;rd++)
                for(int cd=-radius;cd<=radius;cd++)
                {
                     // Get the pixel value
                     //For the functions you write, check the ConvertQImage2Double function to see how to get the pixel value
                     QRgb pixel = buffer.pixel(c + cd + radius, r + rd + radius);

                     // Get the value of the kernel
                     double weight = kernel[(rd + radius)*size + cd + radius];

                     rgb[0] += weight*(double) qRed(pixel);
                     rgb[1] += weight*(double) qGreen(pixel);
                     rgb[2] += weight*(double) qBlue(pixel);
                }
            // Store the pixel in the image to be returned
            // You need to store the RGB values in the double form of the image
            image->setPixel(c, r, restrictColor(rgb[0],rgb[1],rgb[2]));
        }
    }
    // Clean up (use this carefully)
    delete[] kernel;
}

// Convert QImage to a matrix of size (imageWidth*imageHeight)*3 having double values
void MainWindow::ConvertQImage2Double(QImage image)
{
    // Global variables to access image width and height
    imageWidth = image.width();
    imageHeight = image.height();

    // Initialize the global matrix holding the image
    // This is how you will be creating a copy of the original image inside a function
    // Note: 'Image' is of type 'double**' and is declared in the header file (hence global variable)
    // So, when you create a copy (say buffer), write "double** buffer = new double ....."
    Image = new double* [imageWidth*imageHeight];
    for (int i = 0; i < imageWidth*imageHeight; i++)
            Image[i] = new double[3];

    // For each pixel
    for (int r = 0; r < imageHeight; r++)
        for (int c = 0; c < imageWidth; c++)
        {
            // Get a pixel from the QImage form of the image
            QRgb pixel = image.pixel(c,r);

            // Assign the red, green and blue channel values to the 0, 1 and 2 channels of the double form of the image respectively
            Image[r*imageWidth+c][0] = (double) qRed(pixel);
            Image[r*imageWidth+c][1] = (double) qGreen(pixel);
            Image[r*imageWidth+c][2] = (double) qBlue(pixel);
        }
}

// Convert the matrix form of the image back to QImage for display
void MainWindow::ConvertDouble2QImage(QImage *image)
{
    for (int r = 0; r < imageHeight; r++)
        for (int c = 0; c < imageWidth; c++)
            image->setPixel(c, r, restrictColor(Image[r*imageWidth+c][0], Image[r*imageWidth+c][1], Image[r*imageWidth+c][2]));
}


/**************************************************
 TIME TO WRITE CODE
**************************************************/

/**************************************************
 TASK 1
**************************************************/

// Convolve the image with the kernel
void MainWindow::Convolution(double** image, double *kernel, int kernelWidth, int kernelHeight, bool add)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * kernel: 1-D array of kernel values
 * kernelWidth: width of the kernel
 * kernelHeight: height of the kernel
 * add: a boolean variable (taking values true or false)
*/
{
    // Add your code here
    int radius_w = kernelWidth / 2;
    int radius_h = kernelHeight / 2;
    qDebug("w: %d, h: %d", radius_w, radius_h);
    double **newImage = new double* [imageWidth*imageHeight];
    // Image = new double* [imageWidth*imageHeight];
    for (int i = 0; i < imageWidth*imageHeight; i++)
        newImage[i] = new double[3];
    for(int r = 0; r < imageHeight; r++){
        for(int c = 0; c < imageWidth; c++){

            double rgb[3];
            rgb[0] = rgb[1] = rgb[2] = (add == true) ? 128.0 : 0.0;

            for(int rd = -radius_h; rd <= radius_h; rd++){
                for(int cd = -radius_w; cd <=radius_w; cd++){

                    int pixel = (r + rd) * imageWidth + c + cd;
                    double weight = kernel[(rd + radius_h)*kernelWidth + cd + radius_w];
                    int currentR = ((r + rd < 0 || r + rd >= imageHeight) || (c + cd < 0 || c + cd >= imageWidth)) ? 0 : image[pixel][0];
                    int currentG = ((r + rd < 0 || r + rd >= imageHeight) || (c + cd < 0 || c + cd >= imageWidth)) ? 0 : image[pixel][1];
                    int currentB = ((r + rd < 0 || r + rd >= imageHeight) || (c + cd < 0 || c + cd >= imageWidth)) ? 0 : image[pixel][2];
                    rgb[0] += weight*currentR;
                    rgb[1] += weight*currentG;
                    rgb[2] += weight*currentB;

                }
            }

            newImage[r * imageWidth + c][0] = rgb[0];
            newImage[r * imageWidth + c][1] = rgb[1];
            newImage[r * imageWidth + c][2] = rgb[2];
        }
    }
    //refractor memory copy part
    for(int r = 0; r < imageHeight; r++){
        for(int c = 0; c < imageWidth; c++){
            image[r * imageWidth + c][0] = newImage[r * imageWidth + c][0];
            image[r * imageWidth + c][1] = newImage[r * imageWidth + c][1];
            image[r * imageWidth + c][2] = newImage[r * imageWidth + c][2];
        }
    }
}

/**************************************************
 TASK 2
**************************************************/

// Apply the 2-D Gaussian kernel on an image to blur it
void MainWindow::GaussianBlurImage(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    // Add your code here
    // Make Gaussian Blur kernel
    // Use Convolution
    if(sigma == 0)
        return;
    int radius = (int)(ceil(3 * sigma));
    qDebug("radius: %d", radius);
    int size = 2 * radius + 1;
    double *kernel = new double[size*size];
    double s = 2 * sigma * sigma;
    for(int i = -radius; i <= radius; i++){
        for(int j = -radius; j <= radius; j++){
            double r = sqrt(i * i + j * j);
            double val = (exp(-(r * r) / s)) / (s * M_PI);
            kernel[(i + radius) * size + j + radius] = val;
            qDebug("val %d, %d: %f", i, j, kernel[(i + radius) * size + j + radius]);
        }
    }
    NormalizeKernel(kernel, size, size);
    Convolution(image, kernel, size, size, false);
}

/**************************************************
 TASK 3
**************************************************/

// Perform the Gaussian Blur first in the horizontal direction and then in the vertical direction
void MainWindow::SeparableGaussianBlurImage(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    int radius = (int)(ceil(3 * sigma));
    int size = 2 * radius + 1;
    double s = 2 * sigma * sigma;
    // Add your code here
    // Blur horizontal
        // Horizontal Kernel
    double *horizontalKernel = new double[size];
    for(int i = -radius; i <= radius; i++){
        double r = sqrt(i * i);
        double val = exp(-(r * r) / s) / (M_PI * s);
        horizontalKernel[i + radius] = val;
    }
    NormalizeKernel(horizontalKernel, size, 1);
    Convolution(image, horizontalKernel, size, 1, false);

    double *verticalKernel = new double[size];
    for(int j = -radius; j <= radius; j++){
        double r = sqrt(j * j);
        double val = exp(-(r * r) / s) / (M_PI * s);
        verticalKernel[j + radius] = val;
    }
    NormalizeKernel(verticalKernel, 1, size);
    Convolution(image, verticalKernel, 1, size, false);
        // Normalize
        // Convolution
    // Blur vertical
        // vertical kernel
        // Normalize
        // Convolution
}

/********** TASK 4 (a) **********/

// Compute the First derivative of an image along the horizontal direction and then apply Gaussian blur.
void MainWindow::FirstDerivImage_x(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    // Add your code here
    double *kernel_x = new double[3];
    kernel_x[0] = -1.0;
    kernel_x[1] = 0.0;
    kernel_x[2] = 1.0;
    Convolution(image, kernel_x, 3, 1, true);
    // GaussianBlurImage(image, sigma);
}

/********** TASK 4 (b) **********/

// Compute the First derivative of an image along the vertical direction and then apply Gaussian blur.
void MainWindow::FirstDerivImage_y(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    // Add your code here
    double *kernel_y = new double[3];
    kernel_y[0] = -1;
    kernel_y[1] = 0;
    kernel_y[2] = 1;
    Convolution(image, kernel_y, 1, 3, true);
    GaussianBlurImage(image, sigma);
}

/********** TASK 4 (c) **********/

// Compute the Second derivative of an image using the Laplacian operator and then apply Gaussian blur
void MainWindow::SecondDerivImage(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    // Add your code here
    double *laplacianKernel= new double[9];
    laplacianKernel[0] = 0.0;
    laplacianKernel[1] = 1.0;
    laplacianKernel[2] = 0.0;
    laplacianKernel[3] = 1.0;
    laplacianKernel[4] = -4.0;
    laplacianKernel[5] = 1.0;
    laplacianKernel[6] = 0.0;
    laplacianKernel[7] = 1.0;
    laplacianKernel[8] = 0.0;

    Convolution(image, laplacianKernel, 3, 3, true);
    GaussianBlurImage(image, sigma);
}

/**************************************************
 TASK 5
**************************************************/

// Sharpen an image by subtracting the image's second derivative from the original image
void MainWindow::SharpenImage(double** image, double sigma, double alpha)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
 * alpha: constant by which the second derivative image is to be multiplied to before subtracting it from the original image
*/
{
    double **copyImage = new double* [imageWidth * imageHeight];
    for(int i = 0; i < imageWidth * imageHeight; i++){
        copyImage[i] = new double[3];
        copyImage[i][0] = image[i][0];
        copyImage[i][1] = image[i][1];
        copyImage[i][2] = image[i][2];
    }
    // Add your code here
    // copy an original image
    // use one to 2nd derivative
    SecondDerivImage(copyImage, sigma);
    for(int i = 0; i < imageWidth * imageHeight; i++){
        image[i][0] = image[i][0] - alpha * (copyImage[i][0] - 128.0);
        image[i][1] = image[i][1] - alpha * (copyImage[i][1] - 128.0);
        image[i][2] = image[i][2] - alpha * (copyImage[i][2] - 128.0);
    }
    // subtracting 2nd derivative from the original
}

/**************************************************
 TASK 6
**************************************************/

// Display the magnitude and orientation of the edges in an image using the Sobel operator in both X and Y directions
void MainWindow::SobelImage(double** image)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * NOTE: image is grayscale here, i.e., all 3 channels have the same value which is the grayscale value
*/
{
    // Add your code here
    double mag = 0;
    double orien = 0;
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

    double **gxImage = new double*[imageWidth * imageHeight];
    double **gyImage = new double*[imageWidth * imageHeight];
    for(int i = 0; i < imageWidth * imageHeight; i++){
        gxImage[i] = new double[3];
        gyImage[i] = new double[3];
        for(int j = 0; j < 3; j++){
            gxImage[i][j] = image[i][j];
            gyImage[i][j] = image[i][j];
        }
    }

    Convolution(gxImage, sobel_x, 3, 3, false);
    Convolution(gyImage, sobel_y, 3, 3, false);

    // Use the following 3 lines of code to set the image pixel values after computing magnitude and orientation
    // Here 'mag' is the magnitude and 'orien' is the orientation angle in radians to be computed using atan2 function
    // (sin(orien) + 1)/2 converts the sine value to the range [0,1]. Similarly for cosine.
    for(int r = 0; r < imageHeight; r++){
        for(int c = 0; c < imageWidth; c++){
            double gx = gxImage[r * imageWidth + c][0];
            double gy = gyImage[r * imageWidth + c][0];
            double powX = pow(gx, 2.0);
            double powY = pow(gy, 2.0);
            mag = sqrt(powX + powY);
            orien = atan2(gy, gx);
            image[r*imageWidth+c][0] = mag*4.0*((sin(orien) + 1.0)/2.0);
            image[r*imageWidth+c][1] = mag*4.0*((cos(orien) + 1.0)/2.0);
            image[r*imageWidth+c][2] = mag*4.0 - image[r*imageWidth+c][0] - image[r*imageWidth+c][1];
        }
    }
    // image[r*imageWidth+c][0] = mag*4.0*((sin(orien) + 1.0)/2.0);
    // image[r*imageWidth+c][1] = mag*4.0*((cos(orien) + 1.0)/2.0);
    // image[r*imageWidth+c][2] = mag*4.0 - image[r*imageWidth+c][0] - image[r*imageWidth+c][1];
}

/**************************************************
 TASK 7
**************************************************/

// Compute the RGB values at a given point in an image using bilinear interpolation.
void MainWindow::BilinearInterpolation(double** image, double x, double y, double rgb[3])
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * x: x-coordinate (corresponding to columns) of the position whose RGB values are to be found
 * y: y-coordinate (corresponding to rows) of the position whose RGB values are to be found
 * rgb[3]: array where the computed RGB values are to be stored
*/
{
    // Add your code here
    int baseX = floor(x);
    int baseY = floor(y);
    int boundX = baseX + 1;
    int boundY = baseY + 1;

    double dX = boundX - baseX;
    double dY = boundY - baseY;

    double diffFromBaseX = x - baseX;
    double diffFromBaseY = y - baseY;

    double diffFromBoundX = boundX - x;
    double diffFromBoundY = boundY - y;
    if(x < 0 || y < 0){
        qDebug("x, y: %f, %f", x, y);
        qDebug("bases: %d, %d", baseX, baseY);
        qDebug("bounds: %d, %d", boundX, boundY);
    }


    for(int i = 0; i < 3; i++){
        double f11 = (baseX >= 0 && baseX < imageWidth && baseY >= 0 && baseY < imageHeight) ? image[baseY * imageWidth + baseX][i] : 0;
        double f12 = (baseX >= 0 && baseX < imageWidth && boundY >= 0 && boundY < imageHeight) ? image[boundY * imageWidth + baseX][i] : 0;
        double f21 = (boundX >= 0 && boundX < imageWidth && baseY >= 0 && baseY < imageHeight) ? image[baseY * imageWidth + boundX][i] : 0;
        double f22 = (boundX >= 0 && boundX < imageWidth && boundY >= 0 && boundY < imageHeight) ? image[boundY * imageWidth + boundX][i] : 0;
        double r1 = (f11 * diffFromBoundX + f21 * diffFromBaseX) / dX;
        double r2 = (f12 * diffFromBoundX + f22 * diffFromBaseX) / dX;

        rgb[i] = (r1 * diffFromBoundY + r2 * diffFromBaseY) / dY;
        // // rgb[i] = ( f11 * diffFromBoundX * diffFromBoundY +
        //     f12 * diffFromBaseY * diffFromBaseX +
        //     f21 * diffFromBoundX * diffFromBaseY +
        //     f22 * diffFromBaseX * diffFromBaseY) /
        //     (dX * dY);
    }

}

/*******************************************************************************
 Here is the code provided for rotating an image. 'orien' is in degrees.
********************************************************************************/

// Rotating an image by "orien" degrees
void MainWindow::RotateImage(double** image, double orien)

{
    double radians = -2.0*3.141*orien/360.0;

    // Make a copy of the original image and then re-initialize the original image with 0
    double** buffer = new double* [imageWidth*imageHeight];
    for (int i = 0; i < imageWidth*imageHeight; i++)
    {
        buffer[i] = new double [3];
        for(int j = 0; j < 3; j++)
            buffer[i][j] = image[i][j];
        image[i] = new double [3](); // re-initialize to 0
    }

    for (int r = 0; r < imageHeight; r++)
       for (int c = 0; c < imageWidth; c++)
       {
            // Rotate around the center of the image
            double x0 = (double) (c - imageWidth/2);
            double y0 = (double) (r - imageHeight/2);

            // Rotate using rotation matrix
            double x1 = x0*cos(radians) - y0*sin(radians);
            double y1 = x0*sin(radians) + y0*cos(radians);

            x1 += (double) (imageWidth/2);
            y1 += (double) (imageHeight/2);

            double rgb[3];
            BilinearInterpolation(buffer, x1, y1, rgb);

            // Note: "image[r*imageWidth+c] = rgb" merely copies the head pointers of the arrays, not the values
            image[r*imageWidth+c][0] = rgb[0];
            image[r*imageWidth+c][1] = rgb[1];
            image[r*imageWidth+c][2] = rgb[2];
        }
}

/**************************************************
 TASK 8
**************************************************/

// Find the peaks of the edge responses perpendicular to the edges
void MainWindow::FindPeaksImage(double** image, double thres)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * NOTE: image is grayscale here, i.e., all 3 channels have the same value which is the grayscale value
 * thres: threshold value for magnitude
*/
{
    // Add your code here
    // mag Image
    // calculate e1, e2 position & gradient value
    // comparison
    double **magImage = new double *[imageWidth * imageHeight];
    for(int i = 0; i < imageWidth * imageHeight; i++){
        magImage[i] = new double[3];
        for(int j = 0; j < 3; j++){
            magImage[i][j] = image[i][j];
        }
    }

    double mag = 0;
    double orien = 0;
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

    double **gxImage = new double*[imageWidth * imageHeight];
    double **gyImage = new double*[imageWidth * imageHeight];
    for(int i = 0; i < imageWidth * imageHeight; i++){
        gxImage[i] = new double[3];
        gyImage[i] = new double[3];
        for(int j = 0; j < 3; j++){
            gxImage[i][j] = image[i][j];
            gyImage[i][j] = image[i][j];
        }
    }

    Convolution(gxImage, sobel_x, 3, 3, false);
    Convolution(gyImage, sobel_y, 3, 3, false);

    // Use the following 3 lines of code to set the image pixel values after computing magnitude and orientation
    // Here 'mag' is the magnitude and 'orien' is the orientation angle in radians to be computed using atan2 function
    // (sin(orien) + 1)/2 converts the sine value to the range [0,1]. Similarly for cosine.
    for(int r = 0; r < imageHeight; r++){
        for(int c = 0; c < imageWidth; c++){
            double gx = gxImage[r * imageWidth + c][0];
            double gy = gyImage[r * imageWidth + c][0];
            double powX = pow(gx, 2.0);
            double powY = pow(gy, 2.0);
            mag = sqrt(powX + powY);
            // orien = atan2(gy, gx);
            magImage[r*imageWidth+c][0] = mag;
            magImage[r*imageWidth+c][1] = mag;
            magImage[r*imageWidth+c][2] = mag;
        }
    }
    double theta = 135 * M_PI / 180.0;
    for(int r = 0; r < imageHeight; r++){
        for(int c = 0; c < imageWidth; c++){
            double e1x = c + cos(theta);
            double e1y = r + sin(theta);
            double e2x = c - cos(theta);
            double e2y = r - sin(theta);
            double rgb_e1[3];
            double rgb_e2[3];

            BilinearInterpolation(magImage, e1x, e1y, rgb_e1);
            BilinearInterpolation(magImage, e2x, e2y, rgb_e2);
            double e = magImage[r * imageWidth + c][0];
            if(e > thres && e >= rgb_e1[0] && e >= rgb_e2[0]){
                image[r * imageWidth + c][0] = 255;
                image[r * imageWidth + c][1] = 255;
                image[r * imageWidth + c][2] = 255;
            } else {
                image[r * imageWidth + c][0] = 0;
                image[r * imageWidth + c][1] = 0;
                image[r * imageWidth + c][2] = 0;
            }
        }
    }

}

/**************************************************
 TASK 9 (a)
**************************************************/

// Perform K-means clustering on a color image using random seeds
void MainWindow::RandomSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{
    // Add your code here
}

/**************************************************
 TASK 9 (b)
**************************************************/

// Perform K-means clustering on a color image using seeds from the image itself
void MainWindow::PixelSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{
    // Add your code here
}


/**************************************************
 EXTRA CREDIT TASKS
**************************************************/

// Perform K-means clustering on a color image using the color histogram
void MainWindow::HistogramSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{
    // Add your code here
}

// Apply the median filter on a noisy image to remove the noise
void MainWindow::MedianImage(double** image, int radius)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * radius: radius of the kernel
*/
{
    // Add your code here
}

// Apply Bilater filter on an image
void MainWindow::BilateralImage(double** image, double sigmaS, double sigmaI)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigmaS: standard deviation in the spatial domain
 * sigmaI: standard deviation in the intensity/range domain
*/
{
    // Add your code here.  Should be similar to GaussianBlurImage.
}

// Perform the Hough transform
void MainWindow::HoughImage(double** image)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
*/
{
    // Add your code here
}

// Perform smart K-means clustering
void MainWindow::SmartKMeans(double** image)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
*/
{
    // Add your code here
}
