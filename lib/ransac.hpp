#ifndef RANSAC_FIXY_HPP
#define RANSAC_FIXY_HPP

#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>

#include <pcl/pcl_macros.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>


 /** \brief @b SampleConsensusInitialAlignment is an implementation of the initial alignment algorithm described in
   *  section IV of "Fast Point Feature Histograms (FPFH) for 3D Registration," Rusu et al.
   * \author Michael Dixon, Radu B. Rusu
   * \ingroup registration
   */



template <typename PointSource, typename PointTarget, typename FeatureT>
class SampleConsensusInitialAlignment : public pcl::Registration<PointSource, PointTarget>
{
public:
    using pcl::Registration<PointSource, PointTarget>::reg_name_;
    using pcl::Registration<PointSource, PointTarget>::input_;
    using pcl::Registration<PointSource, PointTarget>::indices_;
    using pcl::Registration<PointSource, PointTarget>::target_;
    using pcl::Registration<PointSource, PointTarget>::final_transformation_;
    using pcl::Registration<PointSource, PointTarget>::transformation_;
    using pcl::Registration<PointSource, PointTarget>::corr_dist_threshold_;
    using pcl::Registration<PointSource, PointTarget>::min_number_correspondences_;
    using pcl::Registration<PointSource, PointTarget>::max_iterations_;
    using pcl::Registration<PointSource, PointTarget>::tree_;
    using pcl::Registration<PointSource, PointTarget>::transformation_estimation_;
    using pcl::Registration<PointSource, PointTarget>::converged_;
    using pcl::Registration<PointSource, PointTarget>::getClassName;

    using PointCloudSource = typename pcl::Registration<PointSource, PointTarget>::PointCloudSource;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

    using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget>::PointCloudTarget;

    using PointIndicesPtr = pcl::PointIndices::Ptr;
    using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

    using FeatureCloud = pcl::PointCloud<FeatureT>;
    using FeatureCloudPtr = typename FeatureCloud::Ptr;
    using FeatureCloudConstPtr = typename FeatureCloud::ConstPtr;

    using Ptr = pcl::shared_ptr<SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> >;
    using ConstPtr = pcl::shared_ptr<const SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> >;


    class ErrorFunctor
    {
    public:
        using Ptr = pcl::shared_ptr<ErrorFunctor>;
        using ConstPtr = pcl::shared_ptr<const ErrorFunctor>;

        virtual ~ErrorFunctor() = default;
        virtual float operator () (float d) const = 0;
    };

    class HuberPenalty : public ErrorFunctor
    {
    private:
        HuberPenalty() {}
    public:
        HuberPenalty(float threshold) : threshold_(threshold) {}
        virtual float operator () (float e) const
        {
            if (e <= threshold_)
                return (0.5 * e * e);
            return (0.5 * threshold_ * (2.0 * std::fabs(e) - threshold_));
        }
    protected:
        float threshold_;
    };

    class TruncatedError : public ErrorFunctor
    {
    private:
        TruncatedError() {}
    public:
        ~TruncatedError() {}

        TruncatedError(float threshold) : threshold_(threshold) {}
        float operator () (float e) const override
        {
            if (e <= threshold_)
                return (e / threshold_);
            return (1.0);
        }
    protected:
        float threshold_;
    };

    using ErrorFunctorPtr = typename ErrorFunctor::Ptr;

    using FeatureKdTreePtr = typename pcl::KdTreeFLANN<FeatureT>::Ptr;
    /** \brief Constructor. */
    SampleConsensusInitialAlignment() :
        input_features_(), target_features_(),
        nr_samples_(3), min_sample_distance_(0.0f), k_correspondences_(10),
        feature_tree_(new pcl::KdTreeFLANN<FeatureT>),
        error_functor_()
    {
        reg_name_ = "SampleConsensusInitialAlignment";
        max_iterations_ = 1000;

        // Setting a non-std::numeric_limits<double>::max () value to corr_dist_threshold_ to make it play nicely with TruncatedError
        corr_dist_threshold_ = 100.0f;
        transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget>);
    };

    /** \brief Provide a shared pointer to the source point cloud's feature descriptors
      * \param features the source point cloud's features
      */
    void
        setSourceFeatures(const FeatureCloudConstPtr& features);

    /** \brief Get a pointer to the source point cloud's features */
    inline FeatureCloudConstPtr const
        getSourceFeatures() { return (input_features_); }

    /** \brief Provide a shared pointer to the target point cloud's feature descriptors
      * \param features the target point cloud's features
      */
    void
        setTargetFeatures(const FeatureCloudConstPtr& features);

    /** \brief Get a pointer to the target point cloud's features */
    inline FeatureCloudConstPtr const
        getTargetFeatures() { return (target_features_); }

    /** \brief Set the minimum distances between samples
      * \param min_sample_distance the minimum distances between samples
      */
    void
        setMinSampleDistance(float min_sample_distance) { min_sample_distance_ = min_sample_distance; }

    /** \brief Get the minimum distances between samples, as set by the user */
    float
        getMinSampleDistance() { return (min_sample_distance_); }

    /** \brief Set the number of samples to use during each iteration
      * \param nr_samples the number of samples to use during each iteration
      */
    void
        setNumberOfSamples(int nr_samples) { nr_samples_ = nr_samples; }

    /** \brief Get the number of samples to use during each iteration, as set by the user */
    int
        getNumberOfSamples() { return (nr_samples_); }

    /** \brief Set the number of neighbors to use when selecting a random feature correspondence.  A higher value will
      * add more randomness to the feature matching.
      * \param k the number of neighbors to use when selecting a random feature correspondence.
      */
    void
        setCorrespondenceRandomness(int k) { k_correspondences_ = k; }

    /** \brief Get the number of neighbors used when selecting a random feature correspondence, as set by the user */
    int
        getCorrespondenceRandomness() { return (k_correspondences_); }

    /** \brief Specify the error function to minimize
     * \note This call is optional.  TruncatedError will be used by default
     * \param[in] error_functor a shared pointer to a subclass of SampleConsensusInitialAlignment::ErrorFunctor
     */
    void
        setErrorFunction(const ErrorFunctorPtr& error_functor) { error_functor_ = error_functor; }

    /** \brief Get a shared pointer to the ErrorFunctor that is to be minimized
     * \return A shared pointer to a subclass of SampleConsensusInitialAlignment::ErrorFunctor
     */
    ErrorFunctorPtr
        getErrorFunction() { return (error_functor_); }

    

protected:
    /** \brief Choose a random index between 0 and n-1
      * \param n the number of possible indices to choose from
      */
    inline int
        getRandomIndex(int n) { return (static_cast<int> (n * (rand() / (RAND_MAX + 1.0)))); };

    /** \brief Select \a nr_samples sample points from cloud while making sure that their pairwise distances are
      * greater than a user-defined minimum distance, \a min_sample_distance.
      * \param cloud the input point cloud
      * \param nr_samples the number of samples to select
      * \param min_sample_distance the minimum distance between any two samples
      * \param sample_indices the resulting sample indices
      */
    void
        selectSamples(const PointCloudSource& cloud, int nr_samples, float min_sample_distance,
            std::vector<int>& sample_indices);

    /** \brief For each of the sample points, find a list of points in the target cloud whose features are similar to
      * the sample points' features. From these, select one randomly which will be considered that sample point's
      * correspondence.
      * \param input_features a cloud of feature descriptors
      * \param sample_indices the indices of each sample point
      * \param corresponding_indices the resulting indices of each sample's corresponding point in the target cloud
      */
    void
        findSimilarFeatures(const FeatureCloud& input_features, const std::vector<int>& sample_indices,
            std::vector<int>& corresponding_indices);

    /** \brief An error metric for that computes the quality of the alignment between the given cloud and the target.
      * \param cloud the input cloud
      * \param threshold distances greater than this value are capped
      */
    float
        computeErrorMetric(const PointCloudSource& cloud, float threshold);

    /** \brief Rigid transformation computation method.
      * \param output the transformed input point cloud dataset using the rigid transformation found
      * \param guess The computed transforamtion
      */
    void
        computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess) override;

    /** \brief The source point cloud's feature descriptors. */
    FeatureCloudConstPtr input_features_;

    /** \brief The target point cloud's feature descriptors. */
    FeatureCloudConstPtr target_features_;

    /** \brief The number of samples to use during each iteration. */
    int nr_samples_;

    /** \brief The minimum distances between samples. */
    float min_sample_distance_;

    /** \brief The number of neighbors to use when selecting a random feature correspondence. */
    int k_correspondences_;

    /** \brief The KdTree used to compare feature descriptors. */
    FeatureKdTreePtr feature_tree_;

    ErrorFunctorPtr error_functor_;
public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};


template <typename PointSource, typename PointTarget, typename FeatureT> void
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setSourceFeatures(const FeatureCloudConstPtr& features)
{
    if (features == nullptr || features->empty())
    {
        PCL_ERROR("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
        return;
    }
    input_features_ = features;
}


template <typename PointSource, typename PointTarget, typename FeatureT> void
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setTargetFeatures(const FeatureCloudConstPtr& features)
{
    if (features == nullptr || features->empty())
    {
        PCL_ERROR("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
        return;
    }
    target_features_ = features;
    feature_tree_->setInputCloud(target_features_);
}


template <typename PointSource, typename PointTarget, typename FeatureT> void
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::selectSamples(
        const PointCloudSource& cloud, int nr_samples, float min_sample_distance,
        std::vector<int>& sample_indices)
{
    if (nr_samples > static_cast<int> (cloud.points.size()))
    {
        PCL_ERROR("[pcl::%s::selectSamples] ", getClassName().c_str());
        PCL_ERROR("The number of samples (%d) must not be greater than the number of points (%lu)!\n",
            nr_samples, cloud.points.size());
        return;
    }

    // Iteratively draw random samples until nr_samples is reached
    int iterations_without_a_sample = 0;
    int max_iterations_without_a_sample = static_cast<int> (3 * cloud.points.size());
    sample_indices.clear();
    while (static_cast<int> (sample_indices.size()) < nr_samples)
    {
        // Choose a sample at random
        int sample_index = getRandomIndex(static_cast<int> (cloud.points.size()));

        // Check to see if the sample is 1) unique and 2) far away from the other samples
        bool valid_sample = true;
        for (const int& sample_idx : sample_indices)
        {
            float distance_between_samples = euclideanDistance(cloud.points[sample_index], cloud.points[sample_idx]);

            if (sample_index == sample_idx || distance_between_samples < min_sample_distance)
            {
                valid_sample = false;
                break;
            }
        }

        // If the sample is valid, add it to the output
        if (valid_sample)
        {
            sample_indices.push_back(sample_index);
            iterations_without_a_sample = 0;
        }
        else
            ++iterations_without_a_sample;

        // If no valid samples can be found, relax the inter-sample distance requirements
        if (iterations_without_a_sample >= max_iterations_without_a_sample)
        {
            PCL_WARN("[pcl::%s::selectSamples] ", getClassName().c_str());
            PCL_WARN("No valid sample found after %d iterations. Relaxing min_sample_distance_ to %f\n",
                iterations_without_a_sample, 0.5 * min_sample_distance);

            min_sample_distance_ *= 0.5f;
            min_sample_distance = min_sample_distance_;
            iterations_without_a_sample = 0;
        }
    }
}


template <typename PointSource, typename PointTarget, typename FeatureT> void
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::findSimilarFeatures(
        const FeatureCloud& input_features, const std::vector<int>& sample_indices,
        std::vector<int>& corresponding_indices)
{
    std::vector<int> nn_indices(k_correspondences_);
    std::vector<float> nn_distances(k_correspondences_);

    corresponding_indices.resize(sample_indices.size());
    for (std::size_t i = 0; i < sample_indices.size(); ++i)
    {
        // Find the k features nearest to input_features.points[sample_indices[i]]
        feature_tree_->nearestKSearch(input_features, sample_indices[i], k_correspondences_, nn_indices, nn_distances);

        // Select one at random and add it to corresponding_indices
        int random_correspondence = getRandomIndex(k_correspondences_);
        corresponding_indices[i] = nn_indices[random_correspondence];
    }
}


template <typename PointSource, typename PointTarget, typename FeatureT> float
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::computeErrorMetric(
        const PointCloudSource& cloud, float)
{
    std::vector<int> nn_index(1);
    std::vector<float> nn_distance(1);

    const ErrorFunctor& compute_error = *error_functor_;
    float error = 0;

    for (int i = 0; i < static_cast<int> (cloud.points.size()); ++i)
    {
        // Find the distance between cloud.points[i] and its nearest neighbor in the target point cloud
        tree_->nearestKSearch(cloud, i, 1, nn_index, nn_distance);

        // Compute the error
        error += compute_error(nn_distance[0]);
    }
    return (error);
}


template <typename PointSource, typename PointTarget, typename FeatureT> void
    SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)
{
    // Some sanity checks first
    if (!input_features_)
    {
        PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
        PCL_ERROR("No source features were given! Call setSourceFeatures before aligning.\n");
        return;
    }
    if (!target_features_)
    {
        PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
        PCL_ERROR("No target features were given! Call setTargetFeatures before aligning.\n");
        return;
    }

    if (input_->size() != input_features_->size())
    {
        PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
        PCL_ERROR("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
            input_->size(), input_features_->size());
        return;
    }

    if (target_->size() != target_features_->size())
    {
        PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
        PCL_ERROR("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
            target_->size(), target_features_->size());
        return;
    }

    if (!error_functor_)
        error_functor_.reset(new TruncatedError(static_cast<float> (corr_dist_threshold_)));


    std::vector<int> sample_indices(nr_samples_);
    std::vector<int> corresponding_indices(nr_samples_);
    PointCloudSource input_transformed;
    float lowest_error(0);

    final_transformation_ = guess;
    int i_iter = 0;
    converged_ = false;
    if (!guess.isApprox(Eigen::Matrix4f::Identity(), 0.01f))
    {
        // If guess is not the Identity matrix we check it.
        transformPointCloud(*input_, input_transformed, final_transformation_);
        lowest_error = computeErrorMetric(input_transformed, static_cast<float> (corr_dist_threshold_));
        i_iter = 1;
    }

    for (; i_iter < max_iterations_; ++i_iter)
    {
        // Draw nr_samples_ random samples
        selectSamples(*input_, nr_samples_, min_sample_distance_, sample_indices);

        // Find corresponding features in the target cloud
        findSimilarFeatures(*input_features_, sample_indices, corresponding_indices);

        // Estimate the transform from the samples to their corresponding points
        estimateRigidTransformationSVD<PointSource, PointTarget, float>(*input_, sample_indices, *target_, corresponding_indices, transformation_);
        //transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);
        
        // Transform the data and compute the error
        transformPointCloud(*input_, input_transformed, transformation_);
        float error = computeErrorMetric(input_transformed, static_cast<float> (corr_dist_threshold_));

        // If the new error is lower, update the final transformation
        if (i_iter == 0 || error < lowest_error)
        {
            lowest_error = error;
            final_transformation_ = transformation_;
            converged_ = true;
        }
    }

    // Apply the final transformation
    transformPointCloud(*input_, output, final_transformation_);
}


#endif  //#ifndef RANSAC_HPP