#include "mainwindow.h"
#include "math.h"
#include "ui_mainwindow.h"
#include <QtGui>
#include "stdlib.h"
#include <algorithm>


/**************************************************
CODE FOR K-MEANS COLOR IMAGE CLUSTERING (RANDOM SEED)
**************************************************/

void Clustering(QImage *image, int num_clusters, int maxit)
{
        int w = image->width(), h = image->height();
        QImage buffer = image->copy();

        std::vector<QRgb> centers, centers_new;

        //initialize random centers
        int n = 1;
        while (n <= num_clusters)
        {
            QRgb center = qRgb(rand() % 256, rand() % 256, rand() % 256);
            centers.push_back(center);
            centers_new.push_back(center);
            n++;
        }

        //iterative part
        int it = 0;
        std::vector<int> ids;
        while (it < maxit)
        {
                ids.clear();
                //assign pixels to clusters
                for (int r = 0; r < h; r++)
                	for (int c = 0; c < w; c++)
                	{
                        int maxd = 999999, id = 0;
                        for (int n = 0; n < num_clusters; n++)
                        {
                                QRgb pcenter = centers[n];
                                QRgb pnow = buffer.pixel(c, r);
                                int d = abs(qRed(pcenter) - qRed(pnow)) + abs(qGreen(pcenter) - qGreen(pnow)) + abs(qBlue(pcenter) - qBlue(pnow));
                                if (d < maxd)
                                {
                                        maxd = d; id = n;
                                }
                        }
                        ids.push_back(id);
                	}

                //update centers
                std::vector<int> cnt, rs, gs, bs;
                for (int n = 0; n < num_clusters; n++)
                {
                        rs.push_back(0); gs.push_back(0); bs.push_back(0); cnt.push_back(0);
                }
                for (int r = 0; r < h; r++)
                    for (int c = 0; c < w; c++)
                    {
                        QRgb pixel = buffer.pixel(c,r);
                        rs[ids[r * w + c]] += qRed(pixel);
                        gs[ids[r * w + c]] += qGreen(pixel);
                        bs[ids[r * w + c]] += qBlue(pixel);
                        cnt[ids[r * w + c]]++;
                    }
                for (int n = 0; n < num_clusters; n++)
                    if (cnt[n] == 0) // no pixels in a cluster
                        continue;
                    else
                        centers_new[n] = qRgb(rs[n]/cnt[n], gs[n]/cnt[n], bs[n]/cnt[n]);

                centers = centers_new; it++;
        }
        //render results
        for (int r = 0; r < h; r++)
            for (int c = 0; c < w; c++)
                image->setPixel(c, r, qRgb(ids[r * w + c],ids[r * w + c],ids[r * w + c]));
}

/**************************************************
CODE FOR FINDING CONNECTED COMPONENTS
**************************************************/

#include "utils.h"

#define MAX_LABELS 80000

#define I(x,y)   (image[(y)*(width)+(x)])
#define N(x,y)   (nimage[(y)*(width)+(x)])

void uf_union( int x, int y, unsigned int parent[] )
{
    while ( parent[x] )
        x = parent[x];
    while ( parent[y] )
        y = parent[y];
    if ( x != y ) {
        if ( y < x ) parent[x] = y;
        else parent[y] = x;
    }
}

int next_label = 1;

int uf_find( int x, unsigned int parent[], unsigned int label[] )
{
    while ( parent[x] )
        x = parent[x];
    if ( label[x] == 0 )
        label[x] = next_label++;
    return label[x];
}

void conrgn(int *image, int *nimage, int width, int height)
{
    unsigned int parent[MAX_LABELS], labels[MAX_LABELS];
    int next_region = 1, k;

    memset( parent, 0, sizeof(parent) );
    memset( labels, 0, sizeof(labels) );

    for ( int y = 0; y < height; ++y )
    {
        for ( int x = 0; x < width; ++x )
        {
            k = 0;
            if ( x > 0 && I(x-1,y) == I(x,y) )
                k = N(x-1,y);
            if ( y > 0 && I(x,y-1) == I(x,y) && N(x,y-1) < k )
                k = N(x,y-1);
            if ( k == 0 )
            {
                k = next_region; next_region++;
            }
            if ( k >= MAX_LABELS )
            {
                fprintf(stderr, "Maximum number of labels reached. Increase MAX_LABELS and recompile.\n"); exit(1);
            }
            N(x,y) = k;
            if ( x > 0 && I(x-1,y) == I(x,y) && N(x-1,y) != k )
                uf_union( k, N(x-1,y), parent );
            if ( y > 0 && I(x,y-1) == I(x,y) && N(x,y-1) != k )
                uf_union( k, N(x,y-1), parent );
        }
    }
    for ( int i = 0; i < width*height; ++i )
        if ( nimage[i] != 0 )
            nimage[i] = uf_find( nimage[i], parent, labels );

    next_label = 1; // reset its value to its initial value
    return;
}


/**************************************************
 **************************************************
TIME TO WRITE CODE
**************************************************
**************************************************/


/**************************************************
Code to compute the features of a given image (both database images and query image)
**************************************************/

std::vector<double*> MainWindow::ExtractFeatureVector(QImage image)
{
    /********** STEP 1 **********/

    // Display the start of execution of this step in the progress box of the application window
    // You can use these 2 lines to display anything you want at any point of time while debugging

    ui->progressBox->append(QString::fromStdString("Clustering.."));
    QApplication::processEvents();

    // Perform K-means color clustering
    // This time the algorithm returns the cluster id for each pixel, not the rgb values of the corresponding cluster center
    // The code for random seed clustering is provided. You are free to use any clustering algorithm of your choice from HW 1
    // Experiment with the num_clusters and max_iterations values to get the best result

    int num_clusters = 5;
    int max_iterations = 50;
    QImage image_copy = image;
    Clustering(&image_copy,num_clusters,max_iterations);


    /********** STEP 2 **********/


    ui->progressBox->append(QString::fromStdString("Connecting components.."));
    QApplication::processEvents();

    // Find connected components in the labeled segmented image
    // Code is given, you don't need to change

    int r, c, w = image_copy.width(), h = image_copy.height();
    int *img = (int*)malloc(w*h * sizeof(int));
    memset( img, 0, w * h * sizeof( int ) );
    int *nimg = (int*)malloc(w*h *sizeof(int));
    memset( nimg, 0, w * h * sizeof( int ) );

    for (r=0; r<h; r++)
        for (c=0; c<w; c++)
            img[r*w + c] = qRed(image_copy.pixel(c,r));

    conrgn(img, nimg, w, h);

    int num_regions=0;
    for (r=0; r<h; r++)
        for (c=0; c<w; c++)
            num_regions = (nimg[r*w+c]>num_regions)? nimg[r*w+c]: num_regions;

    ui->progressBox->append(QString::fromStdString("#regions = "+std::to_string(num_regions)));
    QApplication::processEvents();

    // The resultant image of Step 2 is 'nimg', whose values range from 1 to num_regions

    // WRITE YOUR REGION THRESHOLDING AND REFINEMENT CODE HERE
    int validRegions = 0;
    int threshold = 1000;
    // for(int i = 0; i < num_regions; i++){
    //     if(nimg[i] < threshold)
    //         lostRegions++;
    // }
    // num_regions -= lostRegions;
    // for()
    int * region_idx;
    region_idx = (int *)malloc((num_regions) * sizeof(int));
    for(int i = 0; i < num_regions; i++)
        region_idx[i] = 0;
    for(int i = 0; i < w; i++){
        for(int j = 0; j < h; j++){
            region_idx[nimg[j*w + i]]++;
        }
    }

    for(int i = 0; i < num_regions; i++){
        if(region_idx[i] > threshold)
            validRegions++;
    }
    qDebug("region_num = %d, lost_regions = %d\n",num_regions,validRegions);

    int * region_table;
    region_table = (int *)malloc((validRegions)*sizeof(int));
    for(int i = 0; i < validRegions; i++)
        region_table[i] = 0;

    qDebug("created region table\n ");
    int counter = 0;
    for(int i = 0; i < num_regions; i++){
        if(region_idx[i]>threshold){
            region_table[counter] = i;
            counter++;
        }
    }
    qDebug("creating region_set");
    vector<vector<pair<int, int>>> region_set;
    for(int i = 0; i < validRegions; i++){
        vector<pair<int, int>> pixel_set;
        for(int j = 0; j < h; j++){
            for(int k = 0; k < w; k++){
                if((nimg[j*w+k]==region_table[i])&&(region_idx[nimg[j*w+k]]>threshold)){
                    pixel_set.push_back(make_pair(j, k));
                }
            }
        }
        region_set.push_back((pixel_set));
    }

    // for(int i = 0; i < validRegions; i++){
    //     qDebug("%d region, %d pixels\n",i,region_set[i].size());
    // }
    // qDebug("n: %d", num_regions);
    /********** STEP 3 **********/


    ui->progressBox->append(QString::fromStdString("Extracting features.."));
    QApplication::processEvents();

    // Extract the feature vector of each region

    // Set length of feature vector according to the number of features you plan to use.
    featurevectorlength = 14;

    // Initializations required to compute feature vector

    std::vector<double*> featurevector; // final feature vector of the image; to be returned
    double **features = new double* [validRegions]; // stores the feature vector for each connected component
    for(int i=0;i<validRegions; i++)
        features[i] = new double[featurevectorlength](); // initialize with zeros

    // Sample code for computing the mean RGB values and size of each connected component

    for(int i=0; i<validRegions; i++){
        for (int j=0; j<region_set[i].size(); j++)
        {
            int r = region_set.at(i).at(j).first;
            int c = region_set.at(i).at(j).second;
            features[i][0] += 1; // stores the number of pixels for each connected component
            features[i][1] += (double) qRed(image.pixel(c,r));
            features[i][2] += (double) qGreen(image.pixel(c,r));
            features[i][3] += (double) qBlue(image.pixel(c,r));
        }
    }
    int size = 256;
    double maxEnergy = 0;
    double maxEntropy = 0;
    double maxContrast = 0;
    double maxHomogeneity = 0;

    for(int i = 0; i < validRegions; i++){
        int n = 0;
        qDebug("new region: %d", i);
        int max_x = -1, min_x = w+1;
        int min_y = h+1, max_y = -1;

        double coocMatrix [size][size];
        for(int k = 0; k < size; k++){
            for(int l = 0; l < size; l++){
                // qDebug("matrix %d", coocMatrix[k][l]);
                coocMatrix[k][l] = 0.0;
                // qDebug("matrix %d", coocMatrix[k][l]);
            }
        }
        for(int j = 0; j < region_set[i].size(); j++){
            int r = region_set.at(i).at(j).first;
            int c = region_set.at(i).at(j).second;

            int dr = r + 1;
            int dc = c + 1;

            bool isInRegion = true;
            if(dr >= h || dc >= w)
                isInRegion = false;
            // auto p = make_pair(dr, dc);
            // if(std::find(region_set[i].begin(), region_set[i].end(), p) != region_set[i].end()){
            //     isInRegion = true;
            //     n++;
            // }

            if(isInRegion){
                double rcGrayValue = (double) qRed(image.pixel(c,r)) + (double) qGreen(image.pixel(c,r)) + (double) qBlue(image.pixel(c,r));
                rcGrayValue /= 3.0;
                double drcGrayValue = (double) qRed(image.pixel(dc,dr)) + (double) qGreen(image.pixel(dc,dr)) + (double) qBlue(image.pixel(dc,dr));
                drcGrayValue /= 3.0;
                coocMatrix[(int)floor(rcGrayValue)][(int)floor(drcGrayValue)]++;
                n ++;
            }
            if (r>max_y) max_y = r;
            if (r<min_y) min_y = r;
            if (c>max_x) max_x = c;
            if (c<min_x) min_x = c;
        }
        //normalize the matrix
        qDebug("n: %d", n);
        for(int k = 0; k < size; k++){
            for(int l = 0; l < size; l++){
                // qDebug("matrix %d", coocMatrix[k][l]);
                coocMatrix[k][l] /= n;
                // qDebug("matrix %d", coocMatrix[k][l]);
            }
        }
        double energy = 0;
        double contrast = 0;
        double entropy = 0;
        double homogeneity = 0;
        for(int k = 0; k < size; k++){
            for(int l = 0; l < size; l++){
                energy += (coocMatrix[k][l] * coocMatrix[k][l]);
                contrast += ((k - l) * (k - l) * coocMatrix[k][l]);
                if(coocMatrix[k][l] > 0)
                    entropy -= (coocMatrix[k][l] * log2(coocMatrix[k][l] + 0.0));
                homogeneity += (coocMatrix[k][l] / (1 + abs(k - l)));
                if(energy > maxEnergy)
                    maxEnergy = energy;
                if(contrast > maxContrast)
                    maxContrast = contrast;
                if(homogeneity > maxHomogeneity)
                    maxHomogeneity = homogeneity;
                if(entropy > maxEntropy)
                    maxEntropy = entropy;
            }
        }
        features[i][4] = energy;
        features[i][5] = contrast;
        features[i][6] = entropy;
        features[i][7] = homogeneity;
        features[i][8] = (double)max_x/w;
        features[i][9] = (double)max_y/h;
        features[i][10] = (double)min_x/w;
        features[i][11] = (double)min_y/h;
        features[i][12] = (double)((min_x+max_x)/2)/w;
        features[i][13] = (double)((min_y+max_y)/2)/h;
    }

    for(int i = 0; i < validRegions; i++){
        features[i][4] /= maxEnergy;
        features[i][5] /= maxContrast;
        features[i][6] /= maxEntropy;
        features[i][7] /= maxHomogeneity;
    }

    // for(int i = 0; i < validRegions; i++){
    //     for(int j = 0; j < region_set[i].size(); j++){
    //         // double r = region_set.at(i).at(j).first;
    //         // double c = region_set.at(i).at(j).second;
    //         // double gray_value = (double) qRed(image.pixel(c,r)) + (double) qGreen(image.pixel(c,r)) + (double) qBlue(image.pixel(c,r));
    //         gray_value = (gray_value/3.0)/255.0;
    //         features[i][4] += gray_value*gray_value; //energy
    //         features[i][5] += gray_value*(r/h-c/w)*(r/h-c/w); //constrast
    //         features[i][6] += -gray_value*log2(gray_value); //entropy
    //         features[i][7] += gray_value/(1+fabs(r/h-c/w)); //homogeneity
    //     }
    //     features[i][4] /= region_set[i].size();
    //     features[i][5] /= region_set[i].size();
    //     features[i][6] /= region_set[i].size();
    //     features[i][7] /= region_set[i].size();
    // }
    //
    // for(int i = 0; i < validRegions; i++){
    //     int max_x = -1, min_x = w+1;
    //     int min_y = h+1, max_y = -1;
    //     for(int j = 0; j < region_set[i].size(); j++){
    //         int r = region_set.at(i).at(j).first;
    //         int c = region_set.at(i).at(j).second;
    //         if (r>max_y) max_y = r;
    //         if (r<min_y) min_y = r;
    //         if (c>max_x) max_x = c;
    //         if (c<min_x) min_x = c;
    //     }
    //     features[i][8] = max_x;
    //     features[i][9] = max_y;
    //     features[i][10] = min_x;
    //     features[i][11] = min_y;
    //     features[i][12] = (min_x+max_x)/2;
    //     features[i][13] = (min_y+max_y)/2;
    // }



    // Save the mean RGB and size values as image feature after normalization
    for(int m=0; m<validRegions; m++)
    {
        for(int n=1; n<4; n++)
            features[m][n] /= features[m][0]*255.0;
        features[m][0] /= (double) w*h;
        featurevector.push_back(features[m]);
    }

    // Return the created feature vector
    ui->progressBox->append(QString::fromStdString("***Done***"));
    QApplication::processEvents();
    free(region_idx);
    free(region_table);
    region_set.clear();
    return featurevector;
}


/***** Code to compute the distance between two images *****/

// Function that implements distance measure 1
double distance1(double* vector1, double* vector2, int length)
{
    double dis = 0;
    for(int i = 0; i < length; i++){
        dis += sqrt((vector1[i] - vector2[i]) * (vector1[i] - vector2[i]));
    }
    // default, for trial only; change according to your distance measure
    return dis;
}

// Function that implements distance measure 2
double distance2(double* vector1, double* vector2, int length)
{
    // default, for trial only; change according to your distance measure
    double dis = 0;
    for(int i = 0; i < length; i++){
        dis += abs((vector1[i] - vector2[i]));
    }
    return dis;
    // return ((double) rand() / (double) RAND_MAX);
}

// Function to calculate the distance between two images
// Input argument isOne takes true for distance measure 1 and takes false for distance measure 2

void MainWindow::CalculateDistances(bool isOne)
{
    for(int n=0; n<num_images; n++) // for each image in the database
    {
        distances[n] = 0.0; // initialize to 0 the distance from query image to a database image

        for (int i = 0; i < queryfeature.size(); i++) // for each region in the query image
        {
            double current_distance = (double) RAND_MAX, new_distance;

            for (int j = 0; j < databasefeatures[n].size(); j++) // for each region in the current database image
            {
                if (isOne)
                    new_distance = distance1(queryfeature[i], databasefeatures[n][j], featurevectorlength);
                else
                    new_distance = distance2(queryfeature[i], databasefeatures[n][j], featurevectorlength);

                current_distance = std::min(current_distance, new_distance); // distance between the best matching regions
            }

            distances[n] = distances[n] + current_distance; // sum of distances between each matching pair of regions
        }

        distances[n] = distances[n] / (double) queryfeature.size(); // normalize by number of matching pairs

        // Display the distance values
        ui->progressBox->append(QString::fromStdString("Distance to image "+std::to_string(n+1)+" = "+std::to_string(distances[n])));
        QApplication::processEvents();
    }
}
