// Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <makula/base/worker_pool.h>
#include <makula/recognition/pcl/localization.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>


using namespace makula::recognition;

Localization::Localization( const std::string file ) :
quit(false),
transformationCounter(0){
     std::ifstream input(file.c_str());
        std::string filename;
        while(input.good())
        {
                std::getline (input, filename);
                if (filename.empty () || filename.at (0) == '#') // Skip blank lines or comments
                        continue;

                pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
                pcl::io::loadPCDFile(filename, *tmp);

                models.push_back(tmp);
        }
        input.close ();
}

Localization::~Localization() {
     
}

void Localization::exit() {
     stopConsuming();
     quit = true;
}

int Localization::main() {
     auto chLocalization = Channel<LocalizationType>();
     auto localizationWorkers = WorkerPool<>([this, &chLocalization]() {
          LocalizationType lt;
          chLocalization >> lt;
          localisation(lt.cluster, lt.best_match);
     });
     
     while(!quit) {
          LocalizationType lt;
          if( takeOn(lt) ) {
               chLocalization << lt;
               localizationWorkers.startWorker();
          } else
               quit = true;
     }
     
     localizationWorkers.wait();
     
     return 0;
}

void Localization::localisation(pcl::PointCloud<PointT>::Ptr& source,const unsigned best_match)
{
        pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr target_downsampled(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);

        Eigen::Matrix4f initial, result, final;

        target = models[best_match - 1];

        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(target);
        sor.setLeafSize (0.01f,0.01f,0.01f);
        sor.filter (*target_downsampled);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures = computeFPFHFeatures(source);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures = computeFPFHFeatures(target_downsampled);

        initial = prealign(targetFeatures, sourceFeatures, target_downsampled, source, tmp);

        result = registrate(source, tmp);

        final = result * initial;

        pcl::transformPointCloud (*target,*output , final);

        std::cout << " -> Found Object of class " << best_match << " with transformation " << std::endl << std::endl;
        std::cout << final << std::endl << std::endl;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Localization::computeFPFHFeatures (pcl::PointCloud<PointT>::Ptr& cloud)
{
        pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfhEst;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree_2 (new pcl::search::KdTree<PointT> ());
        pcl::NormalEstimationOMP<PointT, pcl::Normal> normalEst;

        normalEst.setInputCloud(cloud);
        normalEst.setSearchMethod (tree_2);
        normalEst.setRadiusSearch (0.02);
        normalEst.compute (*normals);

        fpfhEst.setInputCloud (cloud);
        fpfhEst.setInputNormals (normals);
        fpfhEst.setSearchMethod (tree);
        fpfhEst.setRadiusSearch (0.05);
        fpfhEst.compute(*features);

        return features;
}

Eigen::Matrix4f Localization::prealign (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &targetFeatures,
                                        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &sourceFeatures ,
                                        const pcl::PointCloud<PointT>::Ptr &targetCloud,
                                        const pcl::PointCloud<PointT>::Ptr &sourceCloud, pcl::PointCloud<PointT>::Ptr &output)
{
        pcl::SampleConsensusInitialAlignment<PointT,PointT, pcl::FPFHSignature33> sacia;

        sacia.setMinSampleDistance (0.05f);
        sacia.setMaxCorrespondenceDistance (0.1);
        sacia.setMaximumIterations (2000);
        sacia.setInputTarget (sourceCloud);
        sacia.setTargetFeatures (sourceFeatures);
        sacia.setInputCloud (targetCloud);
        sacia.setSourceFeatures (targetFeatures);
        sacia.setRANSACOutlierRejectionThreshold(0.1f);
        sacia.align (*output);

//         std::cout << "SACIA Fitness Score: " << sacia.getFitnessScore () << std::endl;

        return sacia.getFinalTransformation ();
}

Eigen::Matrix4f Localization::registrate(const pcl::PointCloud<PointT>::Ptr &target,
                                         const pcl::PointCloud<PointT>::Ptr &source)
{
        pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        icp.setMaxCorrespondenceDistance (0.1);
        icp.setMaximumIterations (2000);
        icp.setTransformationEpsilon (1e-6);
        icp.setEuclideanFitnessEpsilon(0);
        icp.setInputCloud (source);
        icp.setInputTarget (target);
        icp.setRANSACOutlierRejectionThreshold(0.1f);
        icp.align (*out);

        return icp.getFinalTransformation ();
}
