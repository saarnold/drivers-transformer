require 'transformer/syskit/test'

describe Transformer::SyskitPlugin do
    before do
        Syskit::RobyApp::Plugin.start_local_process_server
        Syskit::RobyApp::Plugin.connect_to_local_process_server
        Roby.app.using_task_library 'test_transformer'
        ::Robot.logger.level = Logger::WARN
        Syskit.conf.use_deployments_from "test_transformer"

        @handler_ids = Syskit::RobyApp::Plugin.plug_engine_in_roby(engine)
    end

    def verify_simple_setup(mission)
        deploy(mission) do
            # Verify that the selection went well
            device_task  = mission.device_child
            process_task = mission.processor_child
            assert_equal "output_test", device_task.selected_frames["dev"],
                "currently selected frames in #{device_task}: #{device_task.selected_frames.inspect}"
            assert_equal "output_test",
                process_task.selected_frames["body"]
            assert_equal "global_output",
                process_task.selected_frames["output"]
        end

        device_task, process_task = mission.device_child, mission.processor_child
        assert_event_emission device_task.start_event
        assert_event_emission process_task.start_event
            
        assert_equal "output_test", process_task.orocos_task.body_frame
        assert_equal "global_output", process_task.orocos_task.output_frame
    end

    def simple_setup(&block)
        test_m = Syskit::Composition.new_submodel :name => 'Test' do
            add TestTransformer::Device, :as => "device"
            add TestTransformer::BasicBehaviour, :as => "processor"
            device_child.connect_to processor_child, :type => :data
        end

        plan.add_mission(
            mission = test_m.
                use(TestTransformer::Device.use_frames("dev" => "output_test")).
                use_frames("output" => "global_output").
                transformer(&block).as_plan)

        mission = mission.as_service
        verify_simple_setup(mission)
        mission
    end

    # Sets up a network with one device and one processor, checks that the
    # frames are set right and that the task is configured properly
    it "should assign the required static transformations" do
        mission = simple_setup do
            static_transform Eigen::Vector3.new(1, 0, 0),
                "output_test" => "global_output"
        end

        execute do
            device_task = mission.device_child
            process_task = mission.processor_child

            statics = process_task.orocos_task.static_transformations
            assert_equal 1, statics.size
            assert_equal Eigen::Vector3.new(1, 0, 0), statics.to_a.first.position
            assert_equal Eigen::Quaternion.Identity, statics.to_a.first.orientation
        end
    end

    def assert_connection_exists(source_task, source_port, sink_task, sink_port)
        assert(source_task.connected_to?(source_port, sink_task.to_task, sink_port))
    end


    # Sets up a network with one device and one processor, checks that the
    # frames are set right, that the task is configured properly and that the
    # dynamic frame producer is connected as it should
    it "should instanciate dynamic producers as required" do
        device_m = Syskit::Device.new_submodel
        TestTransformer::ConfigurableTransformProducer.driver_for device_m, :as => 'driver'
        dev = robot.device(device_m, :as => 'dev').period(0.1)
            
        mission = simple_setup do
            dynamic_transform TestTransformer::ConfigurableTransformProducer.with_arguments("driver_dev" => dev).use_deployments("configurable_transform_producer"),
                "output_test" => "global_output"
        end

        execute do
            device_task  = mission.device_child
            process_task = mission.processor_child
            producer_task = process_task.transformer_output_test2global_output_child

            assert_connection_exists(producer_task, "transform", process_task, "dynamic_transformations")
            assert_equal "output_test", producer_task.orocos_task.from_frame
            assert_equal "global_output", producer_task.orocos_task.to_frame
        end
    end

    # Sets up a network with one device and one processor, checks that the
    # frames are set right, that the task is configured properly and that the
    # dynamic frame producer is connected as it should
    it "should recognize when dynamic producers are connected to dedicated ports" do
        composition_m = Syskit::Composition.new_submodel :name => 'Test' do
            add TestTransformer::Device, :as => "device"
            add TestTransformer::BasicBehaviour, :as => "processor"
            add TestTransformer::ConfigurableTransformProducer, :as => "producer"

            device_child.connect_to processor_child
            producer_child.transform_port.connect_to processor_child.manual_transform_port
        end

        # This setup will check that:
        # * the frame from 'device' is properly propagated to the "from" frame of
        #   the producer through the processor
        # * the "to" frame of the producer is propagated to the world frame of
        #   the processor
        # * the engine recognizes that the manually connected producer is all is
        #   needed, and therefore requires no other static or dynamic frame
        plan.add_mission(
            mission = composition_m.
                use(TestTransformer::Device.use_frames("dev" => "output_test")).
                use(TestTransformer::ConfigurableTransformProducer.use_frames("to" => "global_output")).
                use_deployments("configurable_transform_producer").
                use_frames("output" => "global_output").
                transformer { frames 'output_test', 'global_output' }.
                as_plan)

        verify_simple_setup(mission = mission.as_service)
        execute do
            device_task  = mission.task.device_child
            process_task = mission.task.processor_child

            producer_task = mission.task.producer_child
            assert_equal "output_test", producer_task.orocos_task.from_frame
            assert_equal "global_output", producer_task.orocos_task.to_frame
            assert_equal "global_output", process_task.selected_frames["world"]
        end
    end

    it "should accept being given services as producers" do
        dev_m = Syskit::Device.new_submodel do
            output_port 'trsf', 'base/samples/RigidBodyState'
        end
        TestTransformer::ConfigurableTransformProducer.driver_for dev_m, :as => 'transformation'
        dev = robot.device(dev_m, :as => 'device')
        dev.period 0.1

        req = TestTransformer::ConfigurableTransformProducer.
                with_arguments('transformation_dev' => dev).
                use_deployments('configurable_transform_producer').transformation_srv
        req.dynamics.period 0.1

        mission = simple_setup do
            dynamic_transform req, "output_test" => "global_output"
        end

        execute do
            device_task  = mission.device_child
            process_task = mission.processor_child
            producer_task = process_task.transformer_output_test2global_output_child

            assert_connection_exists(producer_task, "transform", process_task, "dynamic_transformations")
            assert_equal "output_test", producer_task.orocos_task.from_frame
            assert_equal "global_output", producer_task.orocos_task.to_frame
        end
    end
end

