
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_MODULE TestTransformationMaker

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  

#include <Eigen/Geometry>
#include "Transformer.h"
#include <base/samples/laser_scan.h>

using namespace std;

using namespace transformer;


void ls_callback(const base::Time &ts, const base::samples::LaserScan &value, const Transformation &t) {
    std::cout << "Got callback ts: " << ts << " tr " << t.transform.matrix() << std::endl;
}

BOOST_AUTO_TEST_CASE( no_transform_available )
{
    std::cout << "Testcase no transforms available" << std::endl;
    
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    Transformation robot2Laser;
    robot2Laser.from = "robot";
    robot2Laser.to = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    
    
    int r2l_idx = tf.registerTransformationStream("robot", "laser");
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "laser", "robot", &ls_callback, false);
    
//     tf.pushData(r2l_idx, robot2Laser.time, robot2Laser);
    tf.pushData(ls_idx, ls.time, ls);
    

    while(tf.step())
    {
	std::cout << "did step" << std::endl;
    }

}

BOOST_AUTO_TEST_CASE( test1 )
{
    transformer::Transformer tf;
    base::samples::LaserScan ls;
    ls.time = base::Time::fromSeconds(10);
    
    Transformation robot2Laser;
    robot2Laser.from = "robot";
    robot2Laser.to = "laser";
    robot2Laser.time = base::Time::fromSeconds(10);
    robot2Laser.transform = Eigen::Transform3d::Identity();
    
    int r2l_idx = tf.registerTransformationStream("robot", "laser");
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "laser", "robot", &ls_callback, false);

    robot2Laser.time = base::Time::fromSeconds(1);
    tf.pushData(r2l_idx, robot2Laser.time, robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(2);
    tf.pushData(r2l_idx, robot2Laser.time, robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushData(r2l_idx, robot2Laser.time, robot2Laser);

    tf.pushData(r2l_idx, robot2Laser.time, robot2Laser);
    tf.pushData(ls_idx, ls.time, ls);
    

    //build fake chain
    std::vector<TransformationElement *> trChain;
    trChain.push_back(new DynamicTransformationElement(robot2Laser.from, robot2Laser.to, tf.getAggregator(), r2l_idx));
    
    tf.addTransformationChain("laser", "robot", trChain);
    
    while(tf.step())
    {
	std::cout << "did step " << std::endl;
    }
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple )
{
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
    
    int ls_idx = tf.registerDataStream<base::samples::LaserScan>(base::Time::fromMicroseconds(10000), "laser", "robot", &ls_callback, false);
    tf.pushData(ls_idx, ls.time, ls);
    
    
    robot2Laser.time = base::Time::fromSeconds(1);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(2);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(10);
    tf.pushDynamicTransformation(robot2Laser);

    robot2Laser.time = base::Time::fromSeconds(11);
    tf.pushDynamicTransformation(robot2Laser);

    while(tf.step())
    {
	std::cout << "did step " << std::endl;
    }
    
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_simple_inverse )
{
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
	std::cout << "did step " << std::endl;
    }  
}

BOOST_AUTO_TEST_CASE( automatic_chain_generation_complex )
{
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
	std::cout << "did step " << std::endl;
    }  
}


