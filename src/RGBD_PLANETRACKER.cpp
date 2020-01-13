/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/demos/RGBD_PLANETRACKER.hpp>
#include <polyview/image_processing/image_processing.hpp>
#include <polyview/tools/Timer.hpp>
#include <polyview/tools/Statistic.hpp>
#include <polyview/polyview.hpp>
#include <polyview/tools/random_generators.hpp>
#include <polyview/optimization/OptimizationFunctor.hpp>
#include <random>
#include <time.h>



polyview::demos::RGBD_PLANETRACKER::RGBD_PLANETRACKER()
{
  configure();
}

polyview::demos::RGBD_PLANETRACKER::~RGBD_PLANETRACKER()
{}

void
polyview::demos::RGBD_PLANETRACKER::configure()
{
  _state = RGBD_PLANETRACKER::IDLE;
  _p2pDistanceThreshold = 0.05; // 5 cm
  _thresholdNumberPointsOnPlane = 10000;//64000;
  _plot = true;
  _chairThreshold= 0.7;
  _chairDistance= 2.0;//1.5;
  _depthThreshold= 0.07;
  _refinementIterations = 3;
  _chairHeight = 1.0;
  _chairWeidth = 0.52;
  _tlast << 0,0,0;
  _icp = false;
  _maxClass = 48;
  _minClass = 15;
}

double
polyview::demos::RGBD_PLANETRACKER::process(
    polyview::containers::Multiframe::Ptr & mf )
{
    _display.plot(mf->frame(0));

    double track_result = 0.0;

    switch (_state) {
        case IDLE: {
            // initialize points
            bool InitialSuccess = false;
            Eigen::Vector3d p_cam1, p_cam2, p_cam3;
            samplePoints(mf, p_cam1, p_cam2, p_cam3);
            getPlaneHypothesis(p_cam1, p_cam2, p_cam3);

            // verify number of inliers
            int numberGroundPlanePoints = countInliers(mf);
            if (numberGroundPlanePoints > _thresholdNumberPointsOnPlane)
                InitialSuccess = true;

            if (InitialSuccess) {
                for (int k = 0; k < _refinementIterations; k++) {
                    //refinement over all inliers
                    refineInliers(mf, numberGroundPlanePoints);

                    // choose plot points
                    numberGroundPlanePoints = countInliers(mf, k == _refinementIterations - 1 && _plot);
                }

                Eigen::Vector3d normalScaled;
                normalScaled << _normal[0], _normal[1], 10.0 * _normal[2];
//                _display.addPoint(mf->depthFrame(0).camera().worldToCam(normalScaled), io::Exporter::BLUE);

                //move to tracking
                _state = TRACKING;
            }

            break;
        }
        case TRACKING: {
            bool lost = false;

            int numberGroundPlanePoints = countInliers(mf);

            if (numberGroundPlanePoints < _thresholdNumberPointsOnPlane)
                lost = true;
            else {
                for (int k = 0; k < _refinementIterations; k++) {
                    //refinement over all inliers
                    refineInliers(mf, numberGroundPlanePoints);

                    // choose plot points
                    numberGroundPlanePoints = countInliers(mf, k == _refinementIterations - 1 && _plot);
                }

                Eigen::Vector3d normalScaled;
                normalScaled << _normal[0], _normal[1], 10.0 * _normal[2];
//                _display.addPoint(mf->depthFrame(0).camera().worldToCam(normalScaled), io::Exporter::BLUE);
            }

            if (lost)
                _state = IDLE;
            break;
        }
    }

    //initialize Rcan and tcan
    _Rcan.row(2) = _normal.transpose();
    double nynz = sqrt(_normal[1] * _normal[1] + _normal[2] * _normal[2]);
    _Rcan.row(0) << 0.0, -_normal[2] / nynz, _normal[1] / nynz;
    _Rcan.row(1) = _Rcan.row(2).transpose().cross(_Rcan.row(0).transpose()).transpose();
    _tcan << 0.0, 0.0, _depth;

    //calculate R and t
    findObjectOfInterest(mf);
    EigVec curPoints;
    VecVec curClass(_maxClass - _minClass - 1, std::vector<int>(0));
    boundObject(mf, curPoints, curClass);

    if (!_icp) {

        Eigen::Matrix3d Rtemp;
        Eigen::Vector3d ttemp;

        Rtemp << cos(_alpha), -sin(_alpha), 0,
                sin(_alpha), cos(_alpha), 0,
                0, 0, 1;
        ttemp << _CentralPoint[0], _CentralPoint[1], 0;
        _Rbody = Rtemp.transpose();
        _tbody = -Rtemp.transpose() * ttemp;
        _R = _Rbody * _Rcan;
        _t = _Rbody * _tcan + _tbody;

        // reverse the wrong alpha
        if ((_t - _tlast).norm() > 1.0 && _tlast.norm() != 0) {

            Rtemp << cos(_alpha), sin(_alpha), 0,
                    -sin(_alpha), cos(_alpha), 0,
                    0, 0, 1;
            ttemp << _CentralPoint[0], _CentralPoint[1], 0;
            _Rbody = Rtemp.transpose();
            _tbody = -Rtemp.transpose() * ttemp;
            _R = _Rbody * _Rcan;
            _t = _Rbody * _tcan + _tbody;
        }
    }

    // ICP to calculate R and t
    else {
        icp(mf, curPoints, curClass);
    }

    mf->t() = _t;
    mf->R() = _R;
    if(_state == TRACKING)
        _icp = true;
    else
        _icp = false;

    // update last frame parameters
    _refPoints = curPoints;
    _refClass = curClass;
    _tlast = _t;
    _Rlast = _R;
    _Rbodylast = _Rbody;
    _tbodylast = _tbody;


    // choose keyframes
    // 1. get matched points  2. delete updates  3. change plotting part
//    bool makeKeyframe = false;
//    double disparity = 0.0;
//    for (size_t i = 0; i < 100; i++)
//    {
//        int randomIndex = rand() % 100;
//        disparity += (matchedpoints.block(0,randomIndex,2,1) - matchedpoints.block(2,randomIndex,2,1)).norm();
//    }
//    if (disparity/100 > 15)
//        makeKeyframe = true;
//    if(makeKeyframe)
//    {
//        _refPoints = curPoints;
//        _refClass = curClass;
//        _Rbodylast = _Rbody;
//        _tbodylast = _tbody;
//    }

    //plot the bounding box
//    if (_plot)
//    {
//        Eigen::Vector3d pb1;     pb1 << 0.3, 0.7, 0.0;
//        Eigen::Vector3d pb2;     pb2 << -0.7, 0.7, 0.0;
//        Eigen::Vector3d pb3;     pb3 << -0.7, -0.3, 0.0;
//        Eigen::Vector3d pb4;     pb4 << 0.3, -0.3, 0.0;
//        Eigen::Vector3d pu1;     pu1 << 0.3, 0.7, 1.0;
//        Eigen::Vector3d pu2;     pu2 << -0.7, 0.7, 1.0;
//        Eigen::Vector3d pu3;     pu3 << -0.7, -0.3, 1.0;
//        Eigen::Vector3d pu4;     pu4 << 0.3, -0.3, 1.0;
//
//        plotElement(mf, pb1, pb2);     plotElement(mf, pb2, pb3);
//        plotElement(mf, pb3, pb4);     plotElement(mf, pb4, pb1);
//        plotElement(mf, pu1, pu2);     plotElement(mf, pu2, pu3);
//        plotElement(mf, pu3, pu4);     plotElement(mf, pu4, pu1);
//        plotElement(mf, pb1, pu1);     plotElement(mf, pb2, pu2);
//        plotElement(mf, pb3, pu3);     plotElement(mf, pb4, pu4);
//    }

    _display.exportNow();

    return track_result;
}


void
polyview::demos::RGBD_PLANETRACKER::plotElement( containers::Multiframe::Ptr & mf, Eigen::Vector3d & p1, Eigen::Vector3d & p2 )
{
    _display.addLine(
            mf->frame(0).camera().worldToCam(_R.transpose() * (p1-_t)),
            mf->frame(0).camera().worldToCam(_R.transpose() * (p2-_t)),
            io::Exporter::BLUE);
}


void
polyview::demos::RGBD_PLANETRACKER::samplePoints(
        containers::Multiframe::Ptr & mf,
        Eigen::Vector3d & p_cam1, Eigen::Vector3d & p_cam2, Eigen::Vector3d & p_cam3 )
{
    int colOffset = 0;
    int rowOffset = 0;
    while(true)
    {
        mf->depthFrame(0).getPoint(p_cam1,300+rowOffset,100+colOffset);
        mf->depthFrame(0).getPoint(p_cam2,390,600-colOffset);
        mf->depthFrame(0).getPoint(p_cam3,440-rowOffset,320+colOffset);
        //      mf->depthFrame(0).getPoint(p_cam4,300+rowOffset,760+colOffset);

        srand((unsigned)time(NULL));
        colOffset=(rand() % (60))-30;
        rowOffset=(rand() % (60))-30;

        if((abs(p_cam1[2]-p_cam2[2])<_depthThreshold && abs(p_cam2[2]-p_cam3[2]))<_depthThreshold)
            break;
    }
}

void
polyview::demos::RGBD_PLANETRACKER::getPlaneHypothesis(
        Eigen::Vector3d & p_cam1, Eigen::Vector3d & p_cam2, Eigen::Vector3d & p_cam3)
{
    Eigen::Matrix<double,3,4> A;
    A.row(0) << p_cam1.transpose(), 1.0;
    A.row(1) << p_cam2.transpose(), 1.0;
    A.row(2) << p_cam3.transpose(), 1.0;
//      A.row(3) << p_cam4.transpose(), 1.0;

    Eigen::JacobiSVD<Eigen::Matrix<double,3,4> > svd(A,Eigen::ComputeFullV);
    Eigen::Vector4d planeParameters = svd.matrixV().col(3);

    if( planeParameters[3] < 0.0 )
        planeParameters *= -1.0;

    double scale = planeParameters.block<3,1>(0,0).norm();
    planeParameters /= scale;

    _normal = planeParameters.block<3,1>(0,0);
    _depth = planeParameters[3];
}

size_t
polyview::demos::RGBD_PLANETRACKER::countInliers(
      containers::Multiframe::Ptr & mf, bool plotting)
{
    int numberGroundPlanePoints = 0;
    for( size_t r = 0; r < mf->depthFrame(0).img()._height; r++ )
        for( size_t c = 0; c < mf->depthFrame(0).img()._width; c++ )
        {
            Eigen::Vector3d p_cam;
            if( mf->depthFrame(0).getPoint(p_cam,r,c) )
            {//check if on plane
                double pointToPlaneDistance = p_cam.transpose() * _normal + _depth;
                if( fabs(pointToPlaneDistance) < _p2pDistanceThreshold ) {
                    numberGroundPlanePoints++;
                    if (plotting) {
                        Eigen::Vector2d uvCoordinates(c + 0.5, r + 0.5);
                        _display.addPoint(uvCoordinates, io::Exporter::RED);
                    }
                }
            }
        }

    return numberGroundPlanePoints;
}

void
polyview::demos::RGBD_PLANETRACKER::refineInliers(
        containers::Multiframe::Ptr & mf, size_t reserveSize)
{
    Eigen::MatrixXd A2(reserveSize, 4);
    int rowIndex = 0;

    for (size_t r = 0; r < mf->depthFrame(0).img()._height; r++)
        for (size_t c = 0; c < mf->depthFrame(0).img()._width; c++)
        {
            Eigen::Vector3d p_cam;
            if (mf->depthFrame(0).getPoint(p_cam, r, c))
            {
                //check if on plane
                double pointToPlaneDistance = p_cam.transpose() * _normal + _depth;
                if (fabs(pointToPlaneDistance) < _p2pDistanceThreshold )
                    A2.row(rowIndex++) << p_cam.transpose(), 1.0;
            }
        }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A2, Eigen::ComputeFullV);
    Eigen::Vector4d planeParameters = svd.matrixV().col(3);

    if (planeParameters[3] < 0.0)
        planeParameters *= -1.0;

    double scale = planeParameters.block<3, 1>(0, 0).norm();
    planeParameters /= scale;

    _normal = planeParameters.block<3, 1>(0, 0);
    _depth = planeParameters[3];
}


void
polyview::demos::RGBD_PLANETRACKER::findObjectOfInterest(
        containers::Multiframe::Ptr & mf)
{
    int numberROI = 0;
    Eigen::Vector2d SumOfPoint = Eigen::Vector2d::Zero();

    for( size_t r = 0; r < mf->depthFrame(0).img()._height; r++ )
        for( size_t c = 70; c < mf->depthFrame(0).img()._width-70; c++ )
//        for( size_t c = 0; c < mf->depthFrame(0).img()._width; c++ )
        {
            Eigen::Vector3d p_cam;

            if( mf->depthFrame(0).getPoint(p_cam,r,c) )
            {
                Eigen::Vector3d pcan = _Rcan * p_cam + _tcan;

                //check if Region of Interest
                if( pcan[2] > _chairThreshold && pcan.block<2,1>(0,0).norm() < _chairDistance)
                {
                    numberROI++;
                    SumOfPoint  += pcan.block<2,1>(0,0);

//                    if (_plot) {
//                        Eigen::Vector2d uvCoordinates(c + 0.5, r + 0.5);
//                        _display.addPoint(uvCoordinates, io::Exporter::GREEN);
//                    }
                }
            }
        }

    _CentralPoint = SumOfPoint / numberROI;

    Eigen::Matrix2d A3;
    for (size_t r = 0; r < mf->depthFrame(0).img()._height; r++)
        for( size_t c = 70; c < mf->depthFrame(0).img()._width-70; c++ )
//        for (size_t c = 0; c < mf->depthFrame(0).img()._width; c++)
        {
            Eigen::Vector3d p_cam;
            if (mf->depthFrame(0).getPoint(p_cam, r, c))
            {
                Eigen::Vector3d pcan = _Rcan * p_cam + _tcan;

                //check if Region of Interest
                Eigen::Vector2d distance = pcan.block<2,1>(0,0) - _CentralPoint;
                if( pcan[2] > _chairThreshold && pcan.block<2,1>(0,0).norm() < _chairDistance)
                {
                    A3 += distance * distance.transpose();
//                    if (_plot)
//                    {
//                        Eigen::Vector3d projected3dpt;
//                        projected3dpt << pcan.block<2,1>(0,0), 0.0;
//                        Eigen::Vector2d uvCoordinates = mf->depthFrame(0).camera().worldToCam(Rcan.transpose()*(projected3dpt-tcan));
//                        _display.addPoint(uvCoordinates, io::Exporter::YELLOW);
//                    }
                }
            }
        }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(A3, Eigen::ComputeFullV);
    Eigen::Vector2d chairDirection = svd.matrixV().col(0);

    Eigen::Vector2d LineX = _CentralPoint + chairDirection;
    Eigen::Vector2d LineY = _CentralPoint + svd.matrixV().col(1);

    _alpha = atan(chairDirection[1] / chairDirection[0]);

//    if (_plot)
//    {
//        Eigen::Vector3d temp;
//        temp << _CentralPoint, 0.0;
//        Eigen::Vector2d uvCoordinates = mf->depthFrame(0).camera().worldToCam(_Rcan.transpose() * (temp-_tcan));
//        temp << LineX, 0.0;
//        Eigen::Vector2d LineX2d = mf->depthFrame(0).camera().worldToCam(_Rcan.transpose() * (temp-_tcan));
//        temp << LineY, 0.0;
//        Eigen::Vector2d LineY2d = mf->depthFrame(0).camera().worldToCam(_Rcan.transpose() * (temp-_tcan));

//        _display.addLine(uvCoordinates,LineX2d,io::Exporter::ORANGE);
//        _display.addLine(uvCoordinates,LineY2d,io::Exporter::ORANGE);

//    }
}

void
polyview::demos::RGBD_PLANETRACKER::boundObject(
        containers::Multiframe::Ptr & mf, EigVec & Object, VecVec & Class)
{
    //find the chair
    int numberROI = 0;
    double boundstep = 0.02;

    for( size_t r = 0; r < mf->depthFrame(0).img()._height; r++ )
        for( size_t c = 70; c < mf->depthFrame(0).img()._width-70; c++ )
//        for( size_t c = 0; c < mf->depthFrame(0).img()._width; c++ )
        {
            Eigen::Vector3d p_cam;
            if( mf->depthFrame(0).getPoint(p_cam,r,c) )
            {
                Eigen::Vector3d pcan = _Rcan * p_cam + _tcan;

                //check if Region of Interest
                Eigen::Vector2d distance = pcan.block<2,1>(0,0) - _CentralPoint;
                if( pcan[2] > 0.1 && pcan[2] < _chairHeight && distance.norm() < _chairWeidth)
                    numberROI++;
            }
        }

    Object.resize(numberROI);
    numberROI = 0;
    for (size_t r = 0; r < mf->depthFrame(0).img()._height; r++)
        for( size_t c = 70; c < mf->depthFrame(0).img()._width-70; c++ )
//        for (size_t c = 0; c < mf->depthFrame(0).img()._width; c++)
        {
            Eigen::Vector3d p_cam;
            if (mf->depthFrame(0).getPoint(p_cam, r, c)) {
                Eigen::Vector3d pcan = _Rcan * p_cam + _tcan;

                //check if Region of Interest
                Eigen::Vector2d distance = pcan.block<2, 1>(0, 0) - _CentralPoint;
//                if( pcan[2] > _chairThreshold && pcan.block<2,1>(0,0).norm() < _chairDistance)
                if ( pcan[2] > 0.1 && pcan[2] < _chairHeight && distance.norm() < _chairWeidth) {
                    Object[numberROI] = pcan;
                    size_t itera = floor(pcan[2] / boundstep);
                    if(itera < _maxClass && itera > _minClass) {
                        Class[itera - _minClass - 1].push_back(numberROI);
                    }
                    numberROI++;
                    if (_plot) {
                        Eigen::Vector2d uvCoordinates(c + 0.5, r + 0.5);
                        _display.addPoint(uvCoordinates, io::Exporter::GREEN);
                    }
                }
            }
        }
//    for (size_t c = 0; c < _maxClass - _minClass - 1; c++) {
//        for (size_t r = 0; r < Class[c].size(); r++)
//            std::cout << Class[c][r]<<" ";
//        std::cout <<"\n"; }
    }

namespace polyview
{
namespace demos
{

struct ICPcostFunction : optimization::OptimizationFunctor<double>
{
    Eigen::MatrixXd & _A;

    ICPcostFunction( Eigen::MatrixXd & A ) : optimization::OptimizationFunctor<double>(3, 2*A.cols()), _A(A)
    {}

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        assert(x.size() == 3);
        assert((unsigned int) fvec.size() == 2*_A.cols());

        const double delta = 0.05;

        Eigen::Vector2d translation = x.block<2, 1>(0, 0);
        double angle = x[2];
        Eigen::Matrix2d rotation;
        rotation << cos(angle), -sin(angle), sin(angle), cos(angle);

        for (size_t i = 0; i < _A.cols(); i++) {

            Eigen::Vector2d transformedPoint = rotation * _A.block(2,i,2,1) + translation;
            Eigen::Vector2d residuals = _A.block(0,i,2,1) - transformedPoint;

            if( fabs(residuals[0]) < delta )
            {
                fvec[2*i] = residuals[0] / sqrt(2);
            }
            else if( residuals[0] > delta )
            {
                fvec[2*i] =  sqrt( delta * residuals[0] - 0.5 * delta * delta);
            }
            else
            {
                fvec[2*i] = -sqrt(-delta * residuals[0] - 0.5 * delta * delta);
            }

            if( fabs(residuals[1]) < delta )
            {
                fvec[2*i+1] = residuals[1] / sqrt(2);
            }
            else if( residuals[1] > delta )
            {
                fvec[2*i+1] =  sqrt( delta * residuals[1] - 0.5 * delta * delta);
            }
            else
            {
                fvec[2*i+1] = -sqrt(-delta * residuals[1] - 0.5 * delta * delta);
            }
        }

        return 0;
    }
};

}}


void
polyview::demos::RGBD_PLANETRACKER::icp(
        containers::Multiframe::Ptr & mf, EigVec & CurObject, VecVec & CurClass)
{
    Eigen::Matrix3d Rbody = _Rbodylast;
    Eigen::Vector3d tbody = _tbodylast;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    int matchNumbers = 50;
    size_t matchClass = _maxClass - _minClass - 1;

    Eigen::MatrixXd A(5, matchNumbers * matchClass);

    for (size_t iteration = 0; iteration < 50; iteration ++) {

        Eigen::Vector2d ppreSum = Eigen::Vector2d::Zero();
        Eigen::Vector2d pcurSum = Eigen::Vector2d::Zero();
        int count = 0;

        for (size_t r = 0; r < matchClass; r++)
        {
            for (size_t i = 0; i < matchNumbers ; i++) {
                //get random points
                size_t randomIndex = rand() % CurClass[r].size();
                Eigen::Vector3d pcur = Rbody * CurObject[CurClass[r][randomIndex]] + tbody;

//                std::cout << "Cur"<<CurObject[CurClass[r][i]]<<"\n";
//                std::cout << "pre"<<_refPoints[_refClass[r][i]]<<"\n";
//                std::cout << "R" << Rbody <<"\nt\n"<<tbody<<'\n';
                double minDistance = 50.0;
                    for (size_t j = 0; j < _refClass[r].size(); j++) {
                        Eigen::Vector3d ppre = _Rbodylast * _refPoints[_refClass[r][j]] + _tbodylast;

                        Eigen::Vector2d distance = ppre.block<2, 1>(0, 0) - pcur.block<2, 1>(0, 0);
                        if (distance.norm() < minDistance) {
                            minDistance = distance.norm();
                            A.block(0, count, 2, 1) = ppre.block<2, 1>(0, 0);
                            A.block(2, count, 2, 1) = pcur.block<2, 1>(0, 0);
                            A(4, count) = minDistance;
                        }
                    }
                ppreSum += A.block<2, 1>(0, count);
                pcurSum += A.block<2, 1>(2, count);
                count ++;
            }
        }
        //std::cout<<"aaa"<<A<<'\n';

        Eigen::Vector2d ppreCenter = ppreSum / (matchNumbers*matchClass);
        Eigen::Vector2d pcurCenter = pcurSum / (matchNumbers*matchClass);

        int method = 2;
        switch(method)
        {
            case 1:
            {
                Eigen::Matrix2d A2 = Eigen::Matrix2d::Zero();
                for(size_t k = 0; k < matchNumbers; k++)
                    A2 +=  1 / (A(4, k) + 1) *(A.block(2, k, 2, 1) - pcurCenter) * (A.block(0, k, 2, 1) - ppreCenter).transpose();
                // R and t from cur to pre
                Eigen::JacobiSVD<Eigen::Matrix2d> svd(A2, Eigen::ComputeFullU | Eigen::ComputeFullV);

                Eigen::Matrix2d V = svd.matrixV();
                Eigen::Matrix2d U = svd.matrixU();
                Eigen::Matrix2d Rtmp = V * U.transpose();

                //modify the result in case the rotation has determinant=-1
                if( Rtmp.determinant() < 0 )
                {
                    Eigen::Matrix2d V_prime;
                    V_prime.col(0) = V.col(0);
                    V_prime.col(1) = -V.col(1);
                    Rtmp = V_prime * U.transpose();
                }
                Eigen::Vector2d ttmp = ppreCenter - Rtmp * pcurCenter;

                R << Rtmp, Eigen::Vector2d::Zero(),
                        0, 0, 1;
                t << ttmp , 0;
                break;
            }
            case 2:
            {
                const int n=3;
                Eigen::VectorXd x(n);
                x << 0.0, 0.0, 0.0;

                ICPcostFunction functor( A );
                Eigen::NumericalDiff<ICPcostFunction> numDiff(functor);
                Eigen::LevenbergMarquardt< Eigen::NumericalDiff<ICPcostFunction> >
                        lm(numDiff);
                lm.resetParameters();
                lm.parameters.ftol = 1.E4*Eigen::NumTraits<double>::epsilon();
                lm.parameters.xtol = 1.E4*Eigen::NumTraits<double>::epsilon();
                lm.parameters.maxfev = 50;
                lm.minimize(x);

                Eigen::Vector2d ttmp = x.block<2, 1>(0, 0);
                double angle = x[2];
                Eigen::Matrix2d Rtmp;
                Rtmp << cos(angle), -sin(angle), sin(angle), cos(angle);

                R << Rtmp, Eigen::Vector2d::Zero(),
                        0, 0, 1;
                t << ttmp , 0;
                break;
            }
        }

        Rbody = R * Rbody;
        tbody = R * tbody + t;

//        std::cout <<"R:"<< R <<'\n'<<"t:"<< t <<'\n';
//        std::cout <<"Rbody:"<< Rbody <<'\n'<<"tbody:"<< tbody <<'\n';
    }

    _Rbody = Rbody;
    _tbody = tbody;
    _R = Rbody * _Rcan;
    _t = Rbody * _tcan + tbody;
    //std::cout <<"R:"<< R <<'\n'<<"t:"<< t <<'\n';
    //std::cout <<"Rbody:"<< Rbody <<'\n'<<"tbody:"<< tbody <<'\n';
    //std::cout <<"_R:"<< _R <<'\n'<<"_t:"<< _t <<'\n';
}

