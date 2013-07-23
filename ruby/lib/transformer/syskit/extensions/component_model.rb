module Transformer
    # Extensions to the Component class
    module ComponentModelExtension
        # This returns an InstanciatedComponent object that can be used in
        # other #use statements in the deployment spec
        #
        # For instance,
        #
        #   add(Cmp::CorridorServoing).
        #       use(Project::Task.use_frame('body' => 'local_frame'))
        #
        def use_frames(*spec, &block)
            to_instance_requirements.use_frames(*spec, &block)
        end

        # This returns an InstanciatedComponent object that can be used in
        # other #use statements in the deployment spec
        #
        # For instance,
        #
        #   add(Cmp::CorridorServoing).
        #       use(Project::Task.use_transform_producer('body', 'local_frame', Cmp::Actuator.use(Hokuyo::Task)))
        #
        def use_transform_producer(*spec, &block)
            to_instance_requirements.use_transform_producer(*spec, &block)
        end

        # Yields the ports for which a frame is associated, as well as the frame
        # name
        #
        # @yieldparam [Syskit::Models::Port] the port
        # @yieldparam [String] the frame name. This is a task-local name
        def each_annotated_port
            return if !(trsf = transformer)
            trsf.each_annotated_port do |orogen_port, frame_name|
                yield find_port(orogen_port.name), frame_name
            end
        end

        # Yields the ports for which a transformation is associated, as well as
        # the frame name
        #
        # @yieldparam [Syskit::Models::Port] the port
        # @yieldparam [Transform] the associated transformation. This uses
        #   task-local names.
        def each_transform_port
            return if !(trsf = transformer)
            trsf.each_transform_port do |orogen_port, associated_transform|
                yield find_port(orogen_port.name), associated_transform
            end
        end
    end
end

