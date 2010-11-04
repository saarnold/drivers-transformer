
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

Transformation lastTransform;
bool gotCallback;

void ls_callback(const base::Time &ts, const base::samples::LaserScan &value, const Transformation &t) {
    std::cout << "Got callback ts: " << ts << " tr " << t.transform.matrix() << std::endl;
    
     std::cout << "Euler angels : " << t.transform.rotation().eulerAngles(2,1,0).transpose() / M_PI * 180.0 << std::endl;
     lastTransform = t;
     gotCallback = true;
}

void defaultInit() {
    lastTransform = Transformation();
    gotCallback = false;
};

BOOST_AUTO_TEST_CASE( no_chain )
{
    defaultInit();
    std::cout << "Testcase no chain" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromSeconds(10), "laser", "robot", &ls_callback, false);
    tf.pushData(ls_idx, ls.time, ls);    

    while(tf.step())
	;
    
    BOOST_CHECK_EQUAL( gotCallback, false );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple )
{
    defaultInit();
    std::cout << "Testcase automatic chain generation" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    Transformation robot2Laser;
    robot2Laser.from = "robot";
    robot2Laser.to = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.transform = Eigen::Transform3d::Identity();
    robot2Laser.transform.translation() = Eigen::Vector3d(10,0,0);
    
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromSeconds(10), "laser", "robot", &ls_callback, false);
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


    
    while(tf.step())
    {
	;
    }
    
    BOOST_CHECK_EQUAL( gotCallback, true );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple_inverse )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation simple inverse" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    Transformation robot2Laser;
    robot2Laser.from = "robot";
    robot2Laser.to = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.transform = Eigen::Transform3d::Identity();
    robot2Laser.transform.translation() = Eigen::Vector3d(10,0,0);
    
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "robot", "laser", &ls_callback, false);
    tf.pushData(ls_idx, ls.time, ls);    
    
    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushDynamicTransformation(robot2Laser);

    while(tf.step())
    {
	;
    }  
    BOOST_CHECK_EQUAL( gotCallback, true );
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_complex )
{
    defaultInit();
    std::cout << std::endl << "Testcase automatic chain generation complex" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    Transformation robot2Body;
    robot2Body.from = "robot";
    robot2Body.to = "body";
    robot2Body.time = base::Time::fromSeconds(10);
    robot2Body.transform = Eigen::Transform3d::Identity();
    
    Transformation head2Body;
    head2Body.from = "head";
    head2Body.to = "body";
    head2Body.time = base::Time::fromSeconds(10);
    head2Body.transform = Eigen::Transform3d::Identity();

    Transformation head2Laser;
    head2Laser.from = "head";
    head2Laser.to = "laser";
    head2Laser.time = base::Time::fromSeconds(10);
    head2Laser.transform = Eigen::Transform3d::Identity();

    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "robot", "laser", &ls_callback, false);
    tf.pushData(ls_idx, ls.time, ls);    
    
    tf.pushStaticTransformation(robot2Body);
    tf.pushDynamicTransformation(head2Body);
    tf.pushDynamicTransformation(head2Laser);

    while(tf.step())
    {
    }
    BOOST_CHECK_EQUAL( gotCallback, true );
}

BOOST_AUTO_TEST_CASE( interpolate )
{
    defaultInit();
    std::cout << std::endl << "Testcase interpolation" << std::endl;
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromMicroseconds(10000);
    
    Transformation robot2laser;
    robot2laser.from = "robot";
    robot2laser.to = "laser";
    robot2laser.time = base::Time::fromMicroseconds(5000);
    robot2laser.transform = Eigen::Transform3d::Identity();
    

    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "robot", "laser", &ls_callback, true);
    tf.pushData(ls_idx, ls.time, ls);    
    
    tf.pushDynamicTransformation(robot2laser);

    robot2laser.time = base::Time::fromMicroseconds(15000);
    robot2laser.transform.rotate(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    robot2laser.transform.translation() = (Eigen::Vector3d(10, 0 ,0));
    tf.pushDynamicTransformation(robot2laser);
    
    while(tf.step())
    {
    }
    
    BOOST_CHECK_EQUAL( gotCallback, true );

    Eigen::Vector3d eulerAngles = lastTransform.transform.rotation().eulerAngles(2,1,0).transpose() / M_PI * 180.0;
    Eigen::Vector3d translation = lastTransform.transform.translation();
    
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
    
    Transformation robot2laser;
    robot2laser.from = "robot";
    robot2laser.to = "laser";
    robot2laser.time = base::Time::fromMicroseconds(5000);
    robot2laser.transform = Eigen::Transform3d::Identity();
    
    tf.pushDynamicTransformation(robot2laser);

    bool threw(false);
    
    try {	
	int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "robot", "laser", &ls_callback, true);
    } catch (std::runtime_error e) {
	threw = true;
    }

    BOOST_CHECK_EQUAL( threw, true );
}

