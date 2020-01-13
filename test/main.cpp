/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/polyview.hpp>
#include <polyview/io/ConsoleLogger.hpp>
#include <polyview/cameras/CameraSystem.hpp>
#include <polyview/io/KinectStreamer2.hpp>
#include <polyview/io/FrameLoader.hpp>
#include <polyview/io/launchHelpers.hpp>
#include <polyview/io/OpenCVExporter.hpp>
#include <polyview/io/OpenCVFlowController.hpp>
#include <string>
#include <polyview/tools/Timer.hpp>

#include <polyview/demos/RGBD_PLANETRACKER.hpp>

#include <polyview/io/GLwindow.hpp>
#include <polyview/io/Grid.hpp>
#include <polyview/io/Frame.hpp>
#include <polyview/io/PointCloud.hpp>

#include <fstream>
#include <string>
#include <iostream>


int main(int argc, char **argv) {
    polyview::io::Logger *logger = new polyview::io::ConsoleLogger();
    polyview::io::Exporter *debugDisplay = new polyview::io::OpenCVExporter();
    polyview::io::FlowController *flowController = new polyview::io::OpenCVFlowController();
    polyview::init_polyview(0, logger, debugDisplay, flowController);

    //create the frameloader
    polyview::tools::OptionSet optionSet;
    polyview::tools::extractOptions(argc, argv, optionSet);
    polyview::io::FrameLoaderBase::Ptr fl;
    polyview::io::createRGBDFrameLoader(optionSet, fl);

    //create the tracker
    polyview::demos::RGBD_PLANETRACKER planeTracker;

    //creating a timer
    polyview::tools::Timer timer;
    size_t counter = 0;

    //visualization stuff
    polyview::io::OpenCVExporter display;
    polyview::io::GLwindow glWindow(0.7854, 1280, 960);
    Eigen::Vector3d gridPose(0.0f, 0.0f, 0.0f);
    Eigen::Matrix3d gridOrientation = Eigen::Matrix3d::Identity();
    std::shared_ptr<polyview::io::ObjectBase> grid(new polyview::io::Grid(gridPose, gridOrientation, 5.0f));
    double visualScale = 1.0;

    while (true) {
        timer.start();

        for( int i = 0; i < 4; i++)
            polyview::containers::Multiframe::Ptr mf = fl->getNext();
        polyview::containers::Multiframe::Ptr mf = fl->getNext();

        std::cout << mf->frame(0).camera().K();
        timer.stop(std::string("loading new frame"),true);
        //*polyview::GLOG << "Working on frame " << (int) counter << "\n";

        double result = planeTracker.process(mf);
        timer.stop(std::string("processing the frame"));

        //set R and t
        std::vector<std::shared_ptr<polyview::io::ObjectBase> > &scene = glWindow.accessWriteBuffer();
        scene.clear();
        scene.push_back(
                std::shared_ptr<polyview::io::ObjectBase>(new polyview::io::Frame(visualScale * mf->t(), mf->R())));
        scene.push_back(grid);

        //push_back the point cloud
        polyview::io::PointCloud *pointCloudPtr = new polyview::io::PointCloud(visualScale * 0.005f);
        std::shared_ptr<polyview::io::ObjectBase> pointCloud(pointCloudPtr);


        for( size_t i = 0; i < planeTracker._refPoints.size(); i++ )
        {
            Eigen::Vector3d ptworld = planeTracker._Rbody * planeTracker._refPoints[i] + planeTracker._tbody;
            pointCloudPtr->addPoint(visualScale * ptworld);
        }

        scene.push_back(pointCloud);
        glWindow.switchBuffer();
        glWindow.processEvents();

        timer.stop(std::string("visualizing points above 0.5m"));
        timer.printSummary();

        counter++;
        if (counter == fl->numberMultiframes())
            break;
    }

    return 0;
}