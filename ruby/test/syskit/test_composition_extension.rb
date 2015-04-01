require 'transformer/syskit/test'

describe Transformer::CompositionExtension do
    before do
        Roby.app.using_task_library 'test_transformer'
        ::Robot.logger.level = Logger::WARN
        Syskit.conf.use_deployments_from "test_transformer"
    end

    describe "#each_transform_output" do
        it "returns nil frames if they are not selected" do
            cmp_m = Syskit::Composition.new_submodel do
                add TestTransformer::ConfigurableTransformProducer, :as => 'producer'
                export producer_child.transform_port
            end
            task = cmp_m.instanciate(plan)
            assert_equal [[task.transform_port, nil, nil]], task.each_transform_output.to_a
        end
    end
    describe "#find_port_for_transform" do
        it "returns a suitable port if there is one" do
            cmp_m = Syskit::Composition.new_submodel do
                add TestTransformer::ConfigurableTransformProducer, :as => 'producer'
                export producer_child.transform_port
            end
            task = cmp_m.instanciate(plan)
            assert_equal task.transform_port, task.find_port_for_transform('from', 'to')
        end
    end
end
