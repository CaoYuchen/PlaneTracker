/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_DEMOS_RGBD_PLANETRACKER_HPP_
#define POLYVIEW_DEMOS_RGBD_PLANETRACKER_HPP_

#include <stdlib.h>
#include <vector>
#include <Eigen/Eigen>
#include <thread>

#include <polyview/containers/Multiframe.hpp>
#include <polyview/io/OpenCVExporter.hpp>


namespace polyview
{
namespace demos
{


class RGBD_PLANETRACKER
{
public:
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > EigVec;
    typedef std::vector<std::vector<int>> VecVec;
    enum State
    {
      IDLE,
      TRACKING
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBD_PLANETRACKER();
    virtual ~RGBD_PLANETRACKER();

    double process(
        polyview::containers::Multiframe::Ptr & mf );

	State _state;
    polyview::io::OpenCVExporter _display;
	Eigen::Vector3d _normal;
	Eigen::Matrix3d _Rcan;
	Eigen::Vector3d _tcan;
    Eigen::Matrix3d _R;
    Eigen::Vector3d _t;
	Eigen::Matrix3d _Rbody;
	Eigen::Vector3d _tbody;
    Eigen::Vector3d _tlast;
    Eigen::Matrix3d _Rlast;
    Eigen::Vector3d _tbodylast;
    Eigen::Matrix3d _Rbodylast;
	Eigen::Vector2d _CentralPoint;

    double _alpha;
    double _depth;
	double _p2pDistanceThreshold;
    double _depthThreshold;
    double _chairThreshold;
    double _chairDistance;
    double _chairHeight;
    double _chairWeidth;
    bool _plot;
    bool _icp;
	size_t _thresholdNumberPointsOnPlane;
	size_t _refinementIterations;
    size_t _maxClass;
    size_t _minClass;
    VecVec _refClass;
    EigVec _refPoints;

private:
    void configure();
	void samplePoints(containers::Multiframe::Ptr & mf, Eigen::Vector3d & p_cam1, Eigen::Vector3d & p_cam2, Eigen::Vector3d & p_cam3 );
	void getPlaneHypothesis( Eigen::Vector3d & p_cam1, Eigen::Vector3d & p_cam2, Eigen::Vector3d & p_cam3);
	size_t countInliers(containers::Multiframe::Ptr & mf, bool plotting = false);
	void refineInliers( containers::Multiframe::Ptr & mf, size_t reserveSize);
    void findObjectOfInterest(containers::Multiframe::Ptr & mf);
    void plotElement( containers::Multiframe::Ptr & mf, Eigen::Vector3d & p1, Eigen::Vector3d & p2 );
    void boundObject(containers::Multiframe::Ptr & mf, EigVec & Object, VecVec & Class);
    void icp(containers::Multiframe::Ptr & mf, EigVec & CurObject, VecVec & CurClass);
};

}
}

#endif /* POLYVIEW_DEMOS_RGBD_SDVO_HPP_ */