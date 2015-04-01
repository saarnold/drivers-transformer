require 'transformer/test'

class TC_TransformerConfiguration < Minitest::Test
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
        conf.static_transform trans, 'from' => 'to1'
        conf.static_transform rot, 'from' => 'to2'
        conf.static_transform trans, rot, 'from' => 'to3'

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

    def test_it_updates_existing_transformations
        conf.frames "from", "to"

        conf.static_transform Eigen::Vector3.new(0, 0, 0), 'from' => 'to'
        conf.static_transform Eigen::Vector3.new(1, 0, 0), 'from' => 'to'
        assert conf.has_transformation?("from", "to")

        assert conf.transformation_for('from', 'to').translation.approx?(Eigen::Vector3.new(1, 0, 0))
    end

    def test_rejects_wrong_static_transform_definition
        conf.frames "from", "to"

        assert_raises(ArgumentError) do
            # No transformation provided
            conf.static_transform "from" => "to"
        end
        assert_raises(ArgumentError) do
            conf.static_transform 10, 'from' => 'to'
        end
        assert_raises(ArgumentError) do
            conf.static_transform 10, Eigen::Vector3.new(0, 0, 0), 'from' => 'to'
        end
        assert_raises(ArgumentError) do
            conf.static_transform Eigen::Vector3.new(0, 0, 0), Eigen::Quaternion.Identity, 10, 'from' => 'to'
        end
        assert !conf.has_transformation?("from", "to")
    end

    def test_dynamic_transform_definition
        conf.frames "from", "to1", "to2", "to3"

        conf.dynamic_transform "producer1", 'from' => 'to1'
        conf.dynamic_transform "producer2", 'from' => 'to2'
        conf.dynamic_transform "producer3", 'from' => 'to3'

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

        conf.dynamic_transform "producer1", 'from' => 'to1'
        assert_raises(ArgumentError) { conf.dynamic_transform("from", "to2", 10) }
        assert(conf.has_transformation?('from', 'to1'))
        assert(!conf.has_transformation?('from', 'to2'))
    end

    def test_it_updates_dynamic_transformations
        conf.frames "from", "to"

        conf.dynamic_transform 'producer1', 'from' => 'to'
        conf.dynamic_transform 'producer2', 'from' => 'to'
        assert_equal 'producer2', conf.transformation_for('from', 'to').producer
    end
end

