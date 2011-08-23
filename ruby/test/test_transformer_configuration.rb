$LOAD_PATH.unshift(File.expand_path(File.join('..', 'lib'), File.dirname(__FILE__)))
require 'transformer'
require 'test/unit'
require 'eigen'

class TC_TransformerConfiguration < Test::Unit::TestCase
    attr_reader :conf

    def setup
        @conf = Transformer::Configuration.new
    end

    def test_frame_definition
        conf.frames :from, :to
        assert conf.has_frame?('from')
        assert conf.has_frame?('to')
    end

    def test_static_transform_definition
        conf.frames "from", "to1", "to2", "to3"

        trans = Eigen::Vector3.new(0, 2, 0)
        trans_id = Eigen::Vector3.new(0, 0, 0)
        rot   = Eigen::Quaternion.new(0, 1, 0, 0)
        rot_id = Eigen::Quaternion.Identity
        conf.static_transform "from", "to1", trans
        conf.static_transform "from", "to2", rot
        conf.static_transform "from", "to3", trans, rot

        assert conf.has_transformation?("from", "to1")
        assert conf.has_transformation?("from", "to2")
        assert conf.has_transformation?("from", "to3")
            
        trsf = conf.transformation_for("from", "to1")
        assert((trsf.translation - trans).norm < 0.001)
        assert(trsf.rotation.approx?(rot_id, 0.001))
            
        trsf = conf.transformation_for("from", "to2")
        assert((trsf.translation - trans_id).norm < 0.001)
        assert(trsf.rotation.approx?(rot, 0.001))
            
        trsf = conf.transformation_for("from", "to3")
        assert((trsf.translation - trans).norm < 0.001)
        assert(trsf.rotation.approx?(rot, 0.001))
    end

    def test_rejects_duplicate_static_transforms
        conf.frames "from", "to"

        conf.static_transform "from", "to", Eigen::Vector3.new(0, 0, 0)
        assert_raises(ArgumentError) do
            conf.static_transform "from", "to", Eigen::Vector3.new(0, 0, 0)
        end
        assert conf.has_transformation?("from", "to")
            
    end

    def test_rejects_wrong_static_transform_definition
        conf.frames "from", "to"

        assert_raises(ArgumentError) do
            # No transformation provided
            conf.static_transform "from", "to"
        end
        assert_raises(ArgumentError) do
            # Invalid source frame
            conf.static_transform "invalid", "to", Eigen::Vector3.new(0, 0, 0)
        end
        assert_raises(ArgumentError) do
            # Invalid target frame
            conf.static_transform "from", "invalid", Eigen::Vector3.new(0, 0, 0)
        end
        assert_raises(ArgumentError) do
            conf.static_transform "from", "to", 10
        end
        assert_raises(ArgumentError) do
            conf.static_transform "from", "to", 10, Eigen::Vector3.new(0, 0, 0)
        end
        assert_raises(ArgumentError) do
            conf.static_transform "from", "to", Eigen::Vector3.new(0, 0, 0), Eigen::Quaternion.Identity, 10
        end
        assert !conf.has_transformation?("from", "to")
    end

    def test_dynamic_transform_definition
        conf.frames "from", "to1", "to2", "to3"

        conf.dynamic_transform "from", "to1", "producer1"
        conf.dynamic_transform "from", "to2", "producer2"
        conf.dynamic_transform "from", "to3", "producer3"

        assert conf.has_transformation?("from", "to1")
        assert conf.has_transformation?("from", "to2")
        assert conf.has_transformation?("from", "to3")
            
        trsf = conf.transformation_for("from", "to1")
        assert_equal("producer1", trsf.producer)
        trsf = conf.transformation_for("from", "to2")
        assert_equal("producer2", trsf.producer)
        trsf = conf.transformation_for("from", "to3")
        assert_equal("producer3", trsf.producer)
    end

    def test_rejects_invalid_producers
        conf.frames "from", "to1", "to2", "to3"
        checker = lambda do |obj|
            if !obj.kind_of?(String)
                raise ArgumentError, "producers can only be strings"
            end
        end
        conf.checker = Transformer::ConfigurationChecker.new(checker)

        conf.dynamic_transform "from", "to1", "producer1"
        assert_raises(ArgumentError) { conf.dynamic_transform("from", "to2", 10) }
        assert(conf.has_transformation?('from', 'to1'))
        assert(!conf.has_transformation?('from', 'to2'))
    end

    def test_rejects_duplicate_dynamic_transforms
        conf.frames "from", "to"

        conf.dynamic_transform "from", "to", Eigen::Vector3.new(0, 0, 0)
        assert_raises(ArgumentError) do
            conf.dynamic_transform "from", "to", Eigen::Vector3.new(0, 0, 0)
        end
        assert conf.has_transformation?("from", "to")
    end
end

