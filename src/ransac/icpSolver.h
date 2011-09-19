#ifndef RANSAC_ICPSOLVER_H
#define RANSAC_ICPSOLVER_H

#include <cassert>
#include <iostream>
#include <vector>

#include <envire/icp.hpp>

using namespace std;

namespace groupsac  {
namespace estimators  {

/// Fit a 2D line with to a set of points
/// Specifically, find a and b in the model y = ax + b
///
/// Input data must be typed as follow :
/// X0 Y0
/// X1 Y1
/// X... Y ...
/// Internal extractor function allow to extract sampled data.

class icpSolver 
{
  enum { MINIMUM_SAMPLES = 3 };

public :
  typedef std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > T;
  typedef Eigen::Affine3d Model;
  typedef Model ModelType;

  int get_MINIMUM_SAMPLES() const {return MINIMUM_SAMPLES;}

  bool solve(const T & candidates, vector<Model> & model) const
  {
      std::cout << "called solve" << std::endl;
      if( candidates.size() < 3 )
      {
	  std::cout << candidates.size() << std::endl;
	  return false;
      }

      envire::icp::Pairs pairs;
      for( size_t i = 0; i < candidates.size(); i++ )
      {
	  const Eigen::Vector3d& v1 = candidates[i].first;
	  const Eigen::Vector3d& v2 = candidates[i].second;
	  pairs.add( v1, v2, (v2-v1).norm() ); 
      }

      Eigen::Affine3d m = pairs.getTransform();
      // make test
      for( size_t i = 0; i < candidates.size(); i++ )
      {
	  const Eigen::Vector3d& v1 = candidates[i].first;
	  const Eigen::Vector3d& v2 = m * candidates[i].second;
	  std::cout << v1.transpose() << std::endl 
	      << v2.transpose() << std::endl
	      << (v2-v1).norm() << std::endl;
      }


      model.push_back( m );
      return true;
  }

  /**
  * Return the candidates that are estimated as inliers to the best model
  *
  * \param[in] model (The model(s) that fit the data).
  * \param[in] candidates (The input data).
  * \param[in] threshold (To evaluate if the candidates is an Inlier or Outlier)
  *
  * \return The list of point that are considered as inliers
  */
  static vector<int> defaultEvaluator(vector<Model> & model,
                                      const T & candidates,
                                      double threshold)
  {
      std::cout << "called defaultEval " << model.size() 
	  << " " << candidates.size() << std::endl;
    assert(model.size() > 0);
    vector< vector<int> > inliers(model.size());
    int bestIndex = 0;

    for (size_t i = 0; i < model.size(); ++i)
    {
      const Model & modelToTest = model[i];
      for (size_t j = 0; j < candidates.size(); ++j)
      {
        Eigen::Vector3d v1 = candidates[j].first;
	Eigen::Vector3d v2 = modelToTest * candidates[j].second;
        const double dist = (v2-v1).norm();
        if ( dist < threshold)
          inliers[i].push_back(j);
      }
      if ( i > 0 && inliers[bestIndex].size() < inliers[i].size())
      {
        bestIndex = i;
      }
    }
    return inliers[bestIndex];
  }

  /**
    * Extract the sampled indices from the data container.
    *
    * \param[in] data (The input data).
    * \param[in] samples (The indices of data to extract (line or row)).
    *
    * \return The sampled data.
    */
  static T extractor(const T & data, const vector<int> & sampled)
  {
      std::cout << "called extractor " << sampled.size() <<  std::endl;
    T test;
    test.resize( sampled.size() );
    for(size_t i=0; i < sampled.size(); ++i)
      test[i] = data[ sampled[i] ];
    return test;
  }
};

}; // namespace estimators
}; // namespace groupsac

#endif 
