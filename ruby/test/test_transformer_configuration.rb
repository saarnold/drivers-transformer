require 'transformer/test'

module Transformer
    describe Configuration do
        attr_reader :conf

        before do
            @conf = Transformer::Configuration.new
        end

        describe "frames" do
            it "declares frames" do
                conf.frames 'from', 'to'
                assert conf.has_frame?('from')
                assert conf.has_frame?('to')
            end

            it "converts symbols to strings" do
                conf.frames :from
                assert conf.has_frame?('from')
            end
        end

        describe "#validate_static_transform_arguments" do
            it "accepts having only one translation argument" do
                trans = Eigen::Vector3.new(0, 2, 0)
                assert_equal ['from', 'to', trans, Eigen::Quaternion.Identity],
                    conf.validate_static_transform_arguments(trans, 'from' => 'to')
            end

            it "accepts having only one rotation argument" do
                q = Eigen::Quaternion.new(0, 1, 0, 0)
                assert_equal ['from', 'to', Eigen::Vector3.Zero, q],
                    conf.validate_static_transform_arguments(q, 'from' => 'to')
            end

            it "accepts having both translation and rotation" do
                v = Eigen::Vector3.new(0, 2, 0)
                q = Eigen::Quaternion.new(0, 1, 0, 0)
                assert_equal ['from', 'to', v, q],
                    conf.validate_static_transform_arguments(v, q, 'from' => 'to')
            end

            it "rejects empty transformation definitions" do
                assert_raises(ArgumentError) do
                    # No transformation provided
                    conf.validate_static_transform_arguments "from" => "to"
                end
            end

            it "rejects a wrong argument type" do
                assert_raises(ArgumentError) do
                    conf.validate_static_transform_arguments 10, 'from' => 'to'
                end
                assert_raises(ArgumentError) do
                    conf.validate_static_transform_arguments 10, Eigen::Vector3.new(0, 0, 0), 'from' => 'to'
                end
            end

            it "rejects having too many arguments" do
                assert_raises(ArgumentError) do
                    conf.validate_static_transform_arguments Eigen::Vector3.new(0, 0, 0), Eigen::Quaternion.Identity, 10, 'from' => 'to'
                end
            end
        end

        describe "declaration of static transformations" do
            def assert_has_static_transform(from, to, translation, rotation)
                assert conf.has_transformation?(from, to)
                tr = conf.transformation_for(from, to)
                assert translation.approx?(tr.translation)
                assert rotation.approx?(tr.rotation)
            end

            it "registers a validated transformation" do
                conf.frames 'from', 'to'
                trans, from, to = Eigen::Vector3.new(1, 0, 0), 'from', 'to'
                flexmock(conf).should_receive(:validate_static_transform_arguments).
                    with(trans, from => to).
                    once.
                    and_return([from, to, (v = flexmock), flexmock])
                assert_equal v, conf.static_transform(trans, from => to).translation
            end

            it "updates existing transformations" do
                conf.static_transform Eigen::Vector3.new(0, 0, 0), 'from' => 'to'
                conf.static_transform Eigen::Vector3.new(1, 0, 0), 'from' => 'to'
                assert_has_static_transform('from', 'to', Eigen::Vector3.new(1, 0, 0), Eigen::Quaternion.Identity)
            end

        end

        describe "declaration of dynamic transformations" do
            it "declares a dynamic transform" do
                conf.dynamic_transform "producer", 'from' => 'to'
                assert conf.has_transformation?("from", "to")
                trsf = conf.transformation_for("from", "to")
                assert_equal("producer", trsf.producer)
            end

            it "validates the given producers using the configuration checker" do
                checker = lambda do |obj|
                    if !obj.kind_of?(String)
                        raise ArgumentError, "producers can only be strings"
                    end
                end
                conf.checker = Transformer::ConfigurationChecker.new(checker)

                conf.dynamic_transform "producer1", 'from' => 'to1'
                assert_raises(ArgumentError) { conf.dynamic_transform(10, "from" => "to2") }
                assert(conf.has_transformation?('from', 'to1'))
                assert(!conf.has_transformation?('from', 'to2'))
            end

            it "updates existing transformations" do
                conf.dynamic_transform 'producer1', 'from' => 'to'
                conf.dynamic_transform 'producer2', 'from' => 'to'
                assert_equal 'producer2', conf.transformation_for('from', 'to').producer
            end
        end

        describe "declaration of example transformations" do
            it "registers a validated transformation" do
                conf.frames 'from', 'to'
                trans, from, to = Eigen::Vector3.new(1, 0, 0), 'from', 'to'
                flexmock(conf).should_receive(:validate_static_transform_arguments).
                    with(trans, from => to).
                    once.
                    and_return([from, to, (v = flexmock), flexmock])
                assert_equal v, conf.example_transform(trans, from => to).translation
            end
            it "allows to access the example transformation" do
                trans, from, to = Eigen::Vector3.new(1, 0, 0), 'from', 'to'
                conf.example_transform(trans, from => to)
                tr = conf.example_transformation_for(from, to)
                assert trans.approx?(tr.translation)
                assert Eigen::Quaternion.Identity.approx?(tr.rotation)
                assert_equal 'from', tr.from
                assert_equal 'to', tr.to
            end
            it "raises if trying to access an example transformation for an unknown frame" do

                assert_raises(ArgumentError) { conf.example_transformation_for('from', 'to') }
                conf.frames 'from1'
                assert_raises(ArgumentError) { conf.example_transformation_for('from1', 'to1') }
                conf.frames 'to2'
                assert_raises(ArgumentError) { conf.example_transformation_for('from2', 'to2') }
            end
            it "returns identity as example between two known frames" do
                conf.frames 'from', 'to'
                tr = conf.example_transformation_for('from', 'to')
                assert_equal 'from', tr.from
                assert_equal 'to', tr.to
                assert_equal Eigen::Vector3.Zero, tr.translation
                assert Eigen::Quaternion.Identity.approx?(tr.rotation)
            end
        end
    end
end
