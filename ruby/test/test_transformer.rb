$LOAD_PATH.unshift(File.expand_path(File.join('..', 'lib'), File.dirname(__FILE__)))
require 'transformer'
require 'test/unit'
require 'eigen'
require 'pp'

class TC_Transformer < Test::Unit::TestCase
    attr_reader :trsf
    def conf; trsf.conf end

    def setup
        @trsf = Transformer::TransformationManager.new
    end

    def test_simple_transformation_chain
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        transforms = [
            conf.static_transform("body", "servo_low", Eigen::Vector3.new(0, 0, 0)),
            conf.dynamic_transform("servo_low", "servo_high", "dynamixel"),
            conf.static_transform("servo_high", "laser", Eigen::Vector3.new(0, 0, 0)),
            conf.static_transform("laser", "camera", Eigen::Vector3.new(0, 0, 0))
        ]
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, false, false], chain.inversions)
    end

    def test_transformation_chain_with_inversions
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        transforms = [
            conf.static_transform("body", "servo_low", Eigen::Vector3.new(0, 0, 0)),
            conf.dynamic_transform("servo_high", "servo_low", "dynamixel"),
            conf.static_transform("servo_high", "laser", Eigen::Vector3.new(0, 0, 0)),
            conf.static_transform("laser", "camera", Eigen::Vector3.new(0, 0, 0))
        ]
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, true, false], chain.inversions)
    end

    def test_transformation_chain_with_loop_ignored
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        transforms = [
            conf.static_transform("body", "servo_low", Eigen::Vector3.new(0, 0, 0)),
            conf.dynamic_transform("servo_high", "servo_low", "dynamixel"),
            conf.static_transform("servo_high", "laser", Eigen::Vector3.new(0, 0, 0)),
            conf.static_transform("servo_low", "body", Eigen::Vector3.new(0, 0, 0))
        ]
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, true, false], chain.inversions)
    end

    def test_transformation_chain_with_loop_shortest_path
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        transforms = [
            conf.static_transform("body", "servo_low", Eigen::Vector3.new(0, 0, 0)),
            conf.dynamic_transform("servo_high", "servo_low", "dynamixel"),
            conf.static_transform("servo_high", "laser", Eigen::Vector3.new(0, 0, 0)),
            conf.static_transform("servo_high", "body", Eigen::Vector3.new(0, 0, 0)),
            conf.static_transform("laser", "camera", Eigen::Vector3.new(0, 0, 0))
        ]
        chain = trsf.transformation_chain("body", "laser")
        assert_equal([transforms[3], transforms[2]], chain.links)
        assert_equal([true, false], chain.inversions)
    end
end

