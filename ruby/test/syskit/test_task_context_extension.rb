require 'transformer/syskit/test'

describe Transformer::TaskContextExtension do
    before do
        Roby.app.using_task_library 'test_transformer'
        ::Robot.logger.level = Logger::WARN
        Syskit.conf.use_deployments_from "test_transformer"
    end

    describe "#each_associated_port" do
        it "maps local names to global ones" do
            task = TestTransformer::Device.new
            task.select_frame('dev', 'test')
            assert_equal [[task.data_port, 'test']], task.each_annotated_port.to_a
        end
        it "yields ports for which the frame is not yet selected" do
            task = TestTransformer::Device.new
            assert_equal [[task.data_port, nil]], task.each_annotated_port.to_a
        end
    end
    describe "#each_transform_port" do
        def verify_each_transform_port(expected, result)
            assert_equal expected.size, result.size
            result.each do |port, transform|
                valid = expected.any? do |exp_port, from, to|
                    exp_port == port && transform.from == from && transform.to == to
                end
                if !valid
                    flunk("unexpected #{port} #{transform.from} #{transform.to}, expected results: #{expected}")
                end
            end
        end
        it "maps local names to global ones" do
            task = TestTransformer::ConfigurableTransformProducer.new
            task.select_frame('from', 'test')
            transform_ports = task.each_transform_port.to_a
            verify_each_transform_port([[task.transform_port, 'test', nil]], transform_ports)
        end
        it "yields ports for which both transform frame are not yet selected" do
            task = TestTransformer::ConfigurableTransformProducer.new
            transform_ports = task.each_transform_port.to_a
            verify_each_transform_port([[task.transform_port, nil, nil]], transform_ports)
        end
    end
    describe "#each_transform_output" do
        it "returns nil frames if they are not selected" do
            task = TestTransformer::ConfigurableTransformProducer.new
            assert_equal [[task.transform_port, nil, nil]], task.each_transform_output.to_a
        end
        it "returns silently if there is no transformer model" do
            task = TestTransformer::ConfigurableTransformProducer.new
            flexmock(task.model).should_receive(:transformer).once
            assert_equal [], task.each_transform_output.to_a
        end
    end
    describe "#find_port_for_transform" do
        it "returns a suitable port if there is one" do
            task = TestTransformer::ConfigurableTransformProducer.new
            assert_equal task.transform_port, task.find_port_for_transform('from', 'to')
        end
    end
end

