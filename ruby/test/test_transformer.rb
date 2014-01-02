$LOAD_PATH.unshift(File.expand_path(File.join('..', 'lib'), File.dirname(__FILE__)))
require 'transformer'
require 'test/unit'
require 'eigen'
require 'pp'

class TC_Transformer < Test::Unit::TestCase
    attr_reader :trsf, :transforms
    def conf; trsf.conf end

    def setup
        @trsf = Transformer::TransformationManager.new
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        @transforms = [
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"),
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "servo_high" => "laser"),
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "laser" => "camera")
        ]
    end

    def test_simple_transformation_chain
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_low" => "servo_high"))
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, false, false], chain.inversions)
    end

    def test_transformation_chain_with_inversions
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_high" => "servo_low"))
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, true, false], chain.inversions)
    end

    def test_transformation_chain_with_loop_shortest_path
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_high" => "servo_low"))
        transforms << conf.static_transform(Eigen::Vector3.new(0, 0, 0), "servo_high" => "body")
        chain = trsf.transformation_chain("body", "laser")
        assert_equal([transforms[4], transforms[2]], chain.links)
        assert_equal([true, false], chain.inversions)
    end

    def test_dup_isolates_the_sets_of_the_receiver_from_the_new_object
        trsf = Transformer::TransformationManager.new
        conf = trsf.conf
        conf.static_transform Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"
        conf.dynamic_transform 'dynamixel', "servo_high" => "servo_low"
        copy = conf.dup
        conf.clear
        assert_equal 2, copy.transforms.size
        assert_equal 3, copy.frames.size
    end

    def test_dup_isolates_the_static_transformations_of_the_receiver_from_the_new_object
        trsf = Transformer::TransformationManager.new
        conf = trsf.conf
        conf.static_transform Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"
        copy = conf.dup
        conf.transforms.first[1].translation.x = 10
        assert_equal 0, copy.transforms.first[1].translation.x
    end
end

