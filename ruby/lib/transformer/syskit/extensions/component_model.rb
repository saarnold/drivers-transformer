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
    end
end

