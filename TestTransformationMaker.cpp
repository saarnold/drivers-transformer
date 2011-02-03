
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_MODULE TestTransformationMaker

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  

#include <Eigen/Geometry>
#include "Transformer.h"
#include <base/samples/laser_scan.h>
#include <Eigen/SVD>

using namespace std;

using namespace transformer;

TransformationType lastTransform;
bool gotCallback;
bool gotSample;
bool doInterpolation;

void ls_callback(const base::Time &ts, const base::samples::LaserScan &value, const Transformation &t) {
    std::cout << "Got callback ts: " << ts << std::endl;
        
    //std::cout << "Euler angels : " << t.orientation.toRotationMatrix().eulerAngles(2,1,0).transpose() / M_PI * 180.0 << std::endl;
    gotSample = t.get(ts, lastTransform, doInterpolation);
    gotCallback = true;
}

void defaultInit() {
    lastTransform = TransformationType();
    gotCallback = false;
};

BOOST_AUTO_TEST_CASE( no_chain )
{
    defaultInit();
    std::cout << "Testcase no chain" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);

    Transformation &t = tf.registerTransformation("laser", "robot");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromSeconds(10), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    

    gotSample = false;
    
    while(tf.step())
	;
    
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, false );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple )
{
    defaultInit();
    std::cout << "Testcase automatic chain generation" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    tf.setTimeout(base::Time::fromSeconds(5));
    
    TransformationType robot2Laser;
    robot2Laser.sourceFrame = "robot";
    robot2Laser.targetFrame = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.orientation = Eigen::Quaterniond::Identity();
    robot2Laser.position = Eigen::Vector3d(10,0,0);
    
    Transformation &t = tf.registerTransformation("laser", "robot");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromMicroseconds(500), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    
    
    robot2Laser.time = base::Time::fromSeconds(1);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(2);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(9);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(11);
    tf.pushDynamicTransformation(robot2Laser);

    gotCallback = false;
    gotSample = false;
    
    while(tf.step())
    {
	;
    }
    
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple_inverse )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation simple inverse" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    TransformationType robot2Laser;
    robot2Laser.targetFrame = "robot";
    robot2Laser.sourceFrame = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.orientation = Eigen::Quaterniond::Identity();
    robot2Laser.position = Eigen::Vector3d(10,0,0);
    
    Transformation &t = tf.registerTransformation("robot", "laser");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    
    
    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(11);
    tf.pushDynamicTransformation(robot2Laser);

    gotCallback = false;
    gotSample = false;
    doInterpolation = false;
    
    while(tf.step())
    {
	;
    }  
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );
}

void tr_callback(const base::Time &time, const transformer::Transformation &tr)
{
    gotSample = tr.get(time, lastTransform, doInterpolation);
    gotCallback = true;
    std::cout << "Got pure transformation callback" << std::endl;
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple_inverse_only_transform )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation simple inverse transform only" << std::endl;
    transformer::Transformer tf;
    
    TransformationType robot2Laser;
    robot2Laser.sourceFrame = "robot";
    robot2Laser.targetFrame = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.orientation = Eigen::Quaterniond::Identity();
    robot2Laser.position = Eigen::Vector3d(10,0,0);

    Transformation &t = tf.registerTransformation("laser", "robot");
    int tr_idx = tf.registerTransfromCallback(t, &tr_callback);
    
    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushDynamicTransformation(robot2Laser);

    tf.requestTransformationAtTime(tr_idx, base::Time::fromSeconds(11));
    
    gotSample = false;
    doInterpolation = false;
    
    while(tf.step())
    {
	;
    }  
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );
}


BOOST_AUTO_TEST_CASE( automatic_chain_generation_complex )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation complex" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    TransformationType robot2Body;
    robot2Body.sourceFrame = "robot";
    robot2Body.targetFrame = "body";
    robot2Body.time = base::Time::fromSeconds(10);
    robot2Body.orientation = Eigen::Quaterniond::Identity();
    robot2Body.position = Eigen::Vector3d(10,0,0);

    
    TransformationType head2Body;
    head2Body.sourceFrame = "head";
    head2Body.targetFrame = "body";
    head2Body.time = base::Time::fromSeconds(10);
    head2Body.orientation = Eigen::Quaterniond::Identity();
    head2Body.position = Eigen::Vector3d(10,0,0);

    TransformationType head2Laser;
    head2Laser.sourceFrame = "head";
    head2Laser.targetFrame = "laser";
    head2Laser.time = base::Time::fromSeconds(10);
    head2Laser.orientation = Eigen::Quaterniond::Identity();
    head2Laser.position = Eigen::Vector3d(10,0,0);

    Transformation &t = tf.registerTransformation("robot", "laser");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromSeconds(8), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    
//     tf.pushData(ls_idx, base::Time::fromSeconds(11), ls);    
    
    tf.pushStaticTransformation(robot2Body);
    tf.pushDynamicTransformation(head2Body);
    tf.pushDynamicTransformation(head2Laser);

    gotCallback = false;
    gotSample = false;
    
    while(tf.step())
    {
    }
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_complex_remapp )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation complex remapped" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    TransformationType robot2Body;
    robot2Body.sourceFrame = "robot";
    robot2Body.targetFrame = "body";
    robot2Body.time = base::Time::fromSeconds(10);
    robot2Body.orientation = Eigen::Quaterniond::Identity();
    robot2Body.position = Eigen::Vector3d(10,0,0);

    
    TransformationType head2Body;
    head2Body.sourceFrame = "head";
    head2Body.targetFrame = "body";
    head2Body.time = base::Time::fromSeconds(10);
    head2Body.orientation = Eigen::Quaterniond::Identity();
    head2Body.position = Eigen::Vector3d(10,0,0);

    TransformationType head2Laser;
    head2Laser.sourceFrame = "head";
    head2Laser.targetFrame = "laser";
    head2Laser.time = base::Time::fromSeconds(10);
    head2Laser.orientation = Eigen::Quaterniond::Identity();
    head2Laser.position = Eigen::Vector3d(10,0,0);

    Transformation &t = tf.registerTransformation("robot", "horst");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromSeconds(8), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    
//     tf.pushData(ls_idx, base::Time::fromSeconds(11), ls);    
    
    tf.pushStaticTransformation(robot2Body);
    tf.pushDynamicTransformation(head2Body);
    tf.pushDynamicTransformation(head2Laser);

    gotCallback = false;
    gotSample = false;
    
    tf.setFrameMapping("horst", "laser");
    
    while(tf.step())
    {
    }
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );
}

BOOST_AUTO_TEST_CASE( interpolate )
{
    defaultInit();
    std::cout << std::endl << "Testcase interpolation" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromMicroseconds(10000);
    
    TransformationType robot2laser;
    robot2laser.sourceFrame = "robot";
    robot2laser.targetFrame = "laser";
    robot2laser.time = base::Time::fromMicroseconds(5000);
    robot2laser.orientation = Eigen::Quaterniond::Identity();
    robot2laser.position = Eigen::Vector3d(0,0,0);


    Transformation &t = tf.registerTransformation("robot", "laser");
    int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), t, &ls_callback);
    tf.pushData(ls_idx, ls.time, ls);    
    
    tf.pushDynamicTransformation(robot2laser);

    robot2laser.time = base::Time::fromMicroseconds(15000);
    robot2laser.orientation = (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    robot2laser.position = (Eigen::Vector3d(10, 0 ,0));
    tf.pushDynamicTransformation(robot2laser);
    
    gotCallback = false;
    gotSample = false;
    
    doInterpolation = true;
    
    while(tf.step())
    {
    }
    
    BOOST_CHECK_EQUAL( gotCallback, true );
    BOOST_CHECK_EQUAL( gotSample, true );

    Eigen::Vector3d eulerAngles = lastTransform.orientation.toRotationMatrix().eulerAngles(2,1,0).transpose() / M_PI * 180.0;
    Eigen::Vector3d translation = lastTransform.position;
    
//     BOOST_CHECK_EQUAL( lastTransform.time, base::Time::fromMicroseconds(10000) );
    
    BOOST_CHECK_EQUAL( eulerAngles.x(), 45);
    BOOST_CHECK_EQUAL( eulerAngles.y(), 0);
    BOOST_CHECK_EQUAL( eulerAngles.z(), 0);
    
    BOOST_CHECK_EQUAL( translation.x(), 5);
    BOOST_CHECK_EQUAL( translation.y(), 0);
    BOOST_CHECK_EQUAL( translation.z(), 0);

}

BOOST_AUTO_TEST_CASE( register_data_stream_after_dyn_transform )
{
    std::cout << std::endl << "Testcase wrong stream order" << std::endl;
    transformer::Transformer tf;
    
    TransformationType robot2laser;
    robot2laser.sourceFrame = "robot";
    robot2laser.targetFrame = "laser";
    robot2laser.time = base::Time::fromMicroseconds(5000);
    robot2laser.orientation = Eigen::Quaterniond::Identity();
    robot2laser.position = Eigen::Vector3d(10,0,0);

    tf.pushDynamicTransformation(robot2laser);

    bool threw(false);
    
    try {	
	Transformation &t = tf.registerTransformation("laser", "robot");
	int ls_idx = tf.registerDataStreamWithTransform<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), t, &ls_callback);
    } catch (std::runtime_error e) {
	threw = true;
    }

    BOOST_CHECK_EQUAL( threw, true );
}

