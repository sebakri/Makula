/*
 * Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <makula/recognition/pcl/classifying.h>
#include <makula/base/worker_pool.h>
#include <libsvm/svm.h>

using namespace makula::recognition;

Classifying::Classifying ( const std::string& filename ) :
model(loadModel(filename))
{
}

Classifying::~Classifying()
{

}

void Classifying::exit()
{
     stopConsuming();
     stopProducing();
     quit = true;
}

int Classifying::main()
{    
     auto classificationWorkers = WorkerPool<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>([this](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster) {
          double prob = 0.;
          const int best_match = classify(cluster, prob);
          
          if(prob > .5) {
               deliver(LocalizationType { cluster, best_match } );
//                std::cout << " -> Found object of class "  << best_match << "\033[1;32m (probability: " << prob << ") \033[0m" << std::endl;
          }
     });
     
     while(!quit) {
          std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters;
          
          if(takeOn(clusters)) {
               for(auto cluster : clusters) {
                    classificationWorkers.startWorker(cluster);
               }
          }
     }
     
     classificationWorkers.wait();
     return 0;
}

const int Classifying::classify ( const Object &object, double &prob )
{
     int best_match;
     unsigned num_of_features = sizeof ( object.getFeatures()->points[0].histogram ) / sizeof ( float ) + 1;
     unsigned num_of_classes = svm_get_nr_class ( model.get() );
     double *probabilities = new double[num_of_classes];

     svm_node *x = new svm_node[num_of_features];

     //transform in libSVM format
     for ( uint i=0; i < num_of_features - 1; i++ ) {
          x[i].index = i + 1;
          x[i].value = object.getFeatures()->points[0].histogram[i];
     }
     x[num_of_features - 1].index = -1;

     if ( svm_check_probability_model ( model.get() ) == 1 )
          best_match = svm_predict_probability ( model.get(), x, probabilities );
     else
          best_match = svm_predict ( model.get(), x );

     prob = probabilities[best_match - 1]; // labeling is from 1...n, index is from 0...n-1

     return best_match;
}

std::unique_ptr<svm_model> Classifying::loadModel ( const std::string& filename )
{
     std::unique_ptr<svm_model> model ( new svm_model );

     if ( ( model = std::unique_ptr<svm_model> ( svm_load_model ( filename.c_str() ) ) ) == 0 )
          throw std::runtime_error ( std::string ( "File not found: " + filename ) );

     return std::move ( model );
}