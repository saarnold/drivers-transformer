module Transformer
    # Module that extends the TaskContext class itself
    module TaskContextExtension
        include ConcreteComponentExtension

        # The set of static transformations that should be provided to the
        # component at configuration time
        attribute(:static_transforms) { Array.new }

        # Returns the transformation that this port provides, using the actual
        # frames (i.e. not the task-level frames, but the frames actually
        # selected).
        #
        # One or both frames might be nil. The return value is nil if no
        # transform is associated at all with this port
        def find_transform_of_port(port)
            return if !(tr = model.transformer)
            if associated_transform = tr.find_transform_of_port(port)
                from = selected_frames[associated_transform.from]
                to   = selected_frames[associated_transform.to]
                Transform.new(from, to)
            end
        end

        # Yields the task output ports that produce a transformation, along with
        # the selected frames for this port
        #
        # The selected frames might be nil if no transformation has been
        # selected
        def each_transform_output
            if !(tr = model.transformer)
                return
            end

            model.each_output_port do |port|
                if associated_transform = tr.find_transform_of_port(port)
                    from = selected_frames[associated_transform.from]
                    to   = selected_frames[associated_transform.to]
                    yield(port, from, to)
                end
            end
        end

        # Given a port associated with a transformer transformation, assign the
        # given frames to this local transformation
        def select_port_for_transform(port, from, to)
            if port.respond_to?(:name)
                if model.find_output_port(port.name) != port
                    raise ArgumentError, "#{port.name} is not an output port of #{self}"
                end
                port = port.name
            end

            if !(tr = model.transformer)
                tr = model.transformer do
                    transform_output port, from => to
                end
            end

            if !(transform = tr.find_transform_of_port(port))
                transform = tr.transform_output(port, from => to)
            end
            select_frames(transform.from => from, transform.to => to)
        end

        # Adds a test to the can_merge? predicate to avoid merging two tasks
        # that have different frame mappings
        def can_merge?(merged_task)
            if !(merge_result = super)
                return merge_result
            end

            if tr = self.model.transformer
                tr.available_frames.each do |frame_name|
                    this_sel = merged_task.selected_frames[frame_name]
                    if this_sel && (sel = selected_frames[frame_name])
                        if this_sel != sel
                            Syskit::NetworkGeneration.debug { "cannot merge #{merged_task} into #{self}: frame selection for #{frame_name} differs (resp. #{merged_task.selected_frames[frame_name]} and #{sel})" }
                            return false
                        end
                    end
                end
            end
            return true
        end

        # Adds a pass in the merge operation that updates the selected frames
        # mapping with the mappings stored in the merged task
        def merge(merged_task)
            new = merged_task.static_transforms.find_all do |trsf|
                !static_transforms.any? { |t| t.from == trsf.from && t.to == trsf.to }
            end
            static_transforms.concat(new)

            selected_frames.merge!(merged_task.selected_frames) do |k, v1, v2|
                if v1 && v2 && v1 != v2
                    raise FrameSelectionConflictDuringMerge.new(self, merged_task, k, v1, v2),
                        "cannot merge #{merged_task} into #{self} as different frames are selected for #{k}: resp. #{v1} and #{v2}"
                end
                v1 || v2
            end
            super if defined? super
        end

        # Select the given global frame names for task-local frames.
        #
        # Example:
        #
        #    select_frames 'local' => 'global'
        #
        # Raises StaticFrameChangeError if 'local' is a static frame (i.e. no
        # property exists to change it) and 'global' is not its hardcoded value.
        def select_frames(selection)
            if tr = self.model.transformer
                selection.each do |local_frame, global_frame|
                    # If the frame is not configurable, raise
                    if tr.static?(local_frame) && local_frame != global_frame
                        raise StaticFrameChangeError.new(self, local_frame, global_frame), "cannot select #{global_frame} for the local frame #{local_frame} in #{self}, as the component does not support configuring that frame"
                    end
                end
            end
            super
        end

        # Applies the selected frames to the task properties
        def configure
            super if defined? super
            if tr = self.model.transformer
                selected_frames.each do |local_frame, global_frame|
                    if orocos_task.has_property?("#{local_frame}_frame")
                        property("#{local_frame}_frame").write(global_frame)
                    end
                end

                if !static_transforms.empty?
                    orocos_task.static_transformations = static_transforms.map do |trsf|
                        rbs = Types::Base::Samples::RigidBodyState.new
                        rbs.zero!
                        rbs.time = Time.now
                        rbs.sourceFrame = trsf.from
                        rbs.targetFrame = trsf.to
                        rbs.position = trsf.translation
                        rbs.orientation = trsf.rotation
                        rbs
                    end
                end
            end
        end

        module ClassExtension
            # Allows access to the transformer declaration from the Roby task model
            #
            # It can also be used to define transformer specifications on tasks
            # that don't have one (for instance to tie ports to frames)
            def transformer(*args, &block); orogen_model.transformer(*args, &block) end
        end
    end
end
