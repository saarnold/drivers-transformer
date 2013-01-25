module Transformer
    # Module used to extend objects of the class Syskit::Component
    module ComponentExtension
        attribute(:selected_frames) { Hash.new }
        attribute(:transformer) { Transformer::Configuration.new }

        def can_merge?(other)
            if !(result = super)
                return result
            end

            selected_frames.each do |task_frame, selected_frame|
                if other_sel = other.selected_frames[task_frame]
                    if other_sel != selected_frame
                        return false
                    end
                end
            end

            transformer.compatible_with?(other.transformer)
            return true
        end

        # Selects +selected_frame+ for the task's +frame_name+
        #
        # @throws FrameSelectionConflict if a different frame was already selected
        def select_frame(frame_name, selected_frame)
            if current = selected_frames[frame_name]
                if current != selected_frame
                    raise FrameSelectionConflict.new(self, frame_name, current, selected_frame), "cannot select both #{current} and #{selected_frame} for the frame #{frame_name} of #{self}"
                end
            else
                selected_frames[frame_name] = selected_frame
            end
        end

        # Selects a set of frame mappings
        #
        # See #select_frame
        def select_frames(mappings)
            mappings.each do |name, selected_frame|
                select_frame(name, selected_frame)
            end
        end

        # Returns true if the specified transformation is provided through a
        # dedicated port on the task, or if it should be built by the
        # transformer by aggregating information from dynamic_transformations
        #
        # The frame names are actual frame names, not task-local ones
        def find_transformation_input(from, to)
            return if !(tr = model.transformer)
            tr.each_transform_port do |port, transform|
                if port.kind_of?(Orocos::Spec::InputPort)
                    port_from = selected_frames[transform.from]
                    port_to   = selected_frames[transform.to]
                    if port_from == from && port_to == to
                        return port
                    end
                end
            end
            nil
        end

        # Returns true if one of the task's input port is configured to provide
        # the requested transformation
        def has_transformation_input?(from, to)
            !!find_transformation_input(from, to)
        end
    end
end

