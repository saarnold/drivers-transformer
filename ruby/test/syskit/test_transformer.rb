require 'transformer/syskit/test'

describe Transformer do
    include Transformer::SyskitPlugin::SelfTest

    before do
        Roby.app.using_task_library 'test_transformer'
        ::Robot.logger.level = Logger::WARN
        Syskit.conf.use_deployments_from "test_transformer"
    end

    def test_it_detects_conflicts_between_frame_selection_and_device_definition
        dev_m = Syskit::Device.new_submodel do
            output_port 'trsf', 'base/samples/RigidBodyState'
        end
        TestTransformer::ConfigurableTransformProducer.driver_for dev_m, :as => 'transformation'
        dev = robot.device(dev_m, :as => 'device').
            frame_transform('test_from' => 'test_to').
            period(0.1)

        req = dev.to_instance_requirements.
            use_deployments('configurable_transform_producer').
            use_frames('from' => 'other_from')

        plan.add_mission(mission = req.as_plan)
        mission = mission.as_service

        inhibit_fatal_messages do
            assert_raises(Transformer::FrameSelectionConflict) do
                deploy(mission)
            end
        end
    end

    def test_link_task_frame_to_device_frame_using_frame
        device_m = Syskit::Device.new_submodel do
            output_port "data", "test_transformer/Result"
        end
        TestTransformer::BasicBehaviour.driver_for device_m, :as => 'driver'
        dev = robot.device(device_m, :as => 'dev').frame('test_frame')
        plan.add_mission(
            mission = TestTransformer::BasicBehaviour.with_arguments("driver_dev" => dev).as_plan)
        mission = mission.as_service
        deploy(mission, :validate_network => false)
        assert_equal "test_frame", mission.selected_frames['output']
    end

    def test_link_task_frame_to_device_frame_using_transform
        device_m = Syskit::Device.new_submodel do
            output_port "data", "base/samples/RigidBodyState"
        end
        TestTransformer::ConfigurableTransformProducer.driver_for device_m, :as => 'driver'
        dev = robot.device(device_m, :as => 'dev').frame_transform('test_source' => 'test_target')
        plan.add_mission(
            mission = TestTransformer::ConfigurableTransformProducer.with_arguments("driver_dev" => dev).use_deployments('configurable_transform_producer').as_plan)
        mission = mission.as_service
        deploy(mission, :validate_network => false)
        assert_equal "test_source", mission.selected_frames['from']
        assert_equal "test_target", mission.selected_frames['to']
    end
end

